"""
B601 快速 FK 层：pinocchio C++ 数值后端（v14 实时化重构，数学不变）。

背景
----
剖面实测（isaac_check/diag_ctrl_profile.py）：HDQ/TNDQ 链 fk_outputs
（含显式雅可比）~1.5 ms/控制步。本模块把 FK 层的数值计算切到
pinocchio（forwardKinematics + 帧运动学 + 帧雅可比，C++ 实现），
TNDQ/HDQ 表示装配管线与期望侧
（simdata/trajectory_generator._pose_tndq_from_rp_derivatives）完全
镜像——误差系统 (4.1)-(4.5) 与控制律 (5.2) 消费的键逐项不变。

等价性依据
----------
- pin gripper_link 帧 ≡ B601TCPChain 末端约定（含 Rz(B601_TOOL_ANGLE)
  尾变换）：B601PinModel.__init__ 构造时即对账（dp/dtheta < 5e-3），
  且尾变换为常值纯旋转、原点重合 => twist/xi、J、Jdot_qdot、约束
  残差与裸链逐项一致（B601TCPChain 文档条款），pin 帧输出可直接
  进入装配管线；
- 表示差来源仅 DH 表（闭式精确）vs URDF 文本几何（rpy 6 位截断），
  与 config.b601_dynamics.pinocchio_cross_check 同量级（~1e-6）；
- J 行序与约定换算：pin LWA 帧雅可比行序随版本不同（实测 Isaac
  环境 pin 4.0.0 为 [v; ω]，与 Motion (linear, angular) 布局一致），
  构造时用数值微分探针自动判定（_probe_jacobian_order），版本无关；
  且 pin 线速度行是帧原点速度 ṗ，而控制链 vec6 约定（dq_vec6）的
  twist 线部是矩约定 v = ṗ + p×ω（ξ = 2 ẋ x̄* 的对偶部，p 为 DH
  基座系下 TCP 位置），故逐列补 p×ω_i，与 core/kinematics.jacobian
  的 Ad(prefix)s 装配逐项一致；
- Jdot_qdot = 帧经典加速度|_{q̈=0}（式 (3.5)：q̈=0 时 ξ̇ 的 vec6
  即 J̇q̇，免显式构造 J̇）。

模块自检：~/isaacsim/python.sh -m simdata.fk_pinocchio
（随机 (q, q̇) 对账 B601TCPChain.fk_outputs，逐键残差 + 计时）。
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.dq_algebra import dq_vec6
from core.tndq_algebra import (
    twist_from_tndq, twist_dot_from_tndq, unit_constraint_residuals,
)
from simdata.traj_pinocchio import (
    B601PinModel, _qmul, _quat_from_R, pin,
)
from simdata.trajectory_generator import _pose_tndq_from_rp_derivatives


class B601PinFk:
    """pinocchio 快速 FK 层：fk_outputs(q, q_dot) 与
    run_lib.B601TCPChain.fk_outputs(q, qd, q_ddot=None,
    with_jacobian=True) 同键同约定（x_bar/x_breve/x/x_dot/xi/xi_dot/
    Jdot_qdot/c0/c1/c2/J）。位置为 DH 基座系（减 B601_BASE_PREFIX），
    姿态为 gripper_link 物理姿态。"""

    def __init__(self, pin_model=None):
        from config.params import B601_BASE_PREFIX
        # 复用 B601PinModel：reduced 模型（手指锁定）+ 构造时 FK 对账
        self.pin_model = pin_model if pin_model is not None else B601PinModel()
        self.base_prefix = np.asarray(B601_BASE_PREFIX, dtype=float)
        self._j_lin_first = self._probe_jacobian_order()

    def _probe_jacobian_order(self):
        """数值探针判定 pin LWA 帧雅可比行序（版本无关）。

        对单关节做中心差分 FK，取 gripper_link 原点位移 = 该列的
        线速度部分；与 Jf[:3]/Jf[3:] 对账定位线速度行块。返回
        True = [v; ω]（实测 Isaac 环境 pin 4.0.0），False = [ω; v]。
        """
        model = self.pin_model.model
        fid = self.pin_model.frame_id
        data = model.createData()          # 独立 data，不污染主 data
        q = pin.neutral(model)
        Jf = np.asarray(pin.computeFrameJacobian(
            model, data, q, fid, pin.LOCAL_WORLD_ALIGNED))
        k = 1                                # 探针关节（避开基座转动退化向）
        e = np.zeros(model.nq)
        e[k] = 1.0
        h = 1e-6
        d2 = model.createData()
        pin.framesForwardKinematics(model, d2, q + h * e)
        p1 = np.array(d2.oMf[fid].translation)
        pin.framesForwardKinematics(model, d2, q - h * e)
        p2 = np.array(d2.oMf[fid].translation)
        v_num = (p1 - p2) / (2.0 * h)
        if np.linalg.norm(v_num) < 1e-9:
            return False                     # 退化探针：保持默认（后续
                                             #   自检会拦截行序错误）
        if np.linalg.norm(Jf[:3, k] - v_num) < 1e-6 * max(
                1.0, np.linalg.norm(v_num)):
            return True                      # [v; ω]
        if np.linalg.norm(Jf[3:, k] - v_num) < 1e-6 * max(
                1.0, np.linalg.norm(v_num)):
            return False                     # [ω; v]
        raise RuntimeError(
            "pin LWA 帧雅可比行序探针失败：两行块均不匹配数值微分"
            f"（|Jf[:3,k]-v|={np.linalg.norm(Jf[:3, k] - v_num):.2e}，"
            f"|Jf[3:,k]-v|={np.linalg.norm(Jf[3:, k] - v_num):.2e}）")

    def fk_outputs(self, q, q_dot):
        """一次 forwardKinematics(q, q̇, 0) + 帧运动学 -> 全 FK 包。"""
        q = np.asarray(q, dtype=float)
        q_dot = np.asarray(q_dot, dtype=float)
        model, data = self.pin_model.model, self.pin_model.data
        fid = self.pin_model.frame_id

        pin.forwardKinematics(model, data, q, q_dot, np.zeros_like(q))
        # pin 2.7：updateFramePlacements(model, data)；新版改名
        # updateFrameKinematics 并带 ReferenceFrame 参
        try:
            pin.updateFrameKinematics(model, data, pin.LOCAL_WORLD_ALIGNED)
        except (AttributeError, TypeError):
            pin.updateFramePlacements(model, data)

        oMf = data.oMf[fid]
        vf = pin.getFrameVelocity(model, data, fid, pin.LOCAL_WORLD_ALIGNED)
        # 经典加速度（q̈=0）= J̇q̇ 的帧表达（式 (3.5)）
        af = pin.getFrameClassicalAcceleration(
            model, data, fid, pin.LOCAL_WORLD_ALIGNED)

        # 平移：世界系 -> DH 基座系（基座前缀为纯平移，导数不变）
        p = oMf.translation - self.base_prefix
        p_dot = np.asarray(vf.linear, dtype=float)
        p_ddot = np.asarray(af.linear, dtype=float)

        # 姿态导数（与期望侧 traj_pinocchio.evaluate 同公式）
        r = _quat_from_R(oMf.rotation)
        omega = np.asarray(vf.angular, dtype=float)
        omega_dot = np.asarray(af.angular, dtype=float)
        om = np.r_[0.0, omega]
        r_dot = 0.5 * _qmul(om, r)
        r_ddot = (0.5 * _qmul(np.r_[0.0, omega_dot], r)
                  + 0.25 * _qmul(om, _qmul(om, r)))

        x_bar = _pose_tndq_from_rp_derivatives(
            r, r_dot, r_ddot, p, p_dot, p_ddot)
        c0, c1, c2 = unit_constraint_residuals(x_bar)

        # 帧雅可比 -> 控制链 vec6 约定 [ω; v]：行序由构造时数值探针
        # 判定；pin 线部为帧原点速度 ṗ，DQ twist 矩约定需逐列补
        # p×ω_i（v = ṗ + p×ω，ξ = 2 ẋ x̄* 的对偶部展开）
        Jf = np.asarray(pin.computeFrameJacobian(
            model, data, q, fid, pin.LOCAL_WORLD_ALIGNED))
        if self._j_lin_first:
            J_lin, J_ang = Jf[:3], Jf[3:]
        else:
            J_lin, J_ang = Jf[3:], Jf[:3]
        J = np.empty((6, Jf.shape[1]))
        J[:3] = J_ang
        J[3:] = J_lin + np.cross(p, J_ang.T).T

        return {
            "x_bar": x_bar,
            "x_breve": x_bar.to_hdq(),
            "x": x_bar.to_dq(),
            "x_dot": np.array(x_bar.ch[1]),
            "xi": dq_vec6(twist_from_tndq(x_bar)),
            "xi_dot": dq_vec6(twist_dot_from_tndq(x_bar)),
            "Jdot_qdot": dq_vec6(twist_dot_from_tndq(x_bar)),
            "c0": c0, "c1": c1, "c2": c2,
            "J": J,
        }


# ---------------------------------------------------------------------------
# 模块自检：~/isaacsim/python.sh -m simdata.fk_pinocchio
# ---------------------------------------------------------------------------

def _selfcheck(n_samples=10, seed=3):
    import time

    from config.params import B601_DH_TABLE, JOINT_LOWER, JOINT_UPPER
    from core.dq_algebra import dq_rotation, dq_translation
    from experiments.run_lib import B601TCPChain

    chain = B601TCPChain(B601_DH_TABLE)
    fk_pin = B601PinFk()
    rng = np.random.default_rng(seed)
    lo, hi = 0.5 * np.asarray(JOINT_LOWER), 0.5 * np.asarray(JOINT_UPPER)

    e_p = e_r = e_xi = e_xid = e_J = e_Jdq = 0.0
    for _ in range(n_samples):
        q = rng.uniform(lo, hi)
        qd = rng.uniform(-1.0, 1.0, 6)
        a = chain.fk_outputs(q, qd, q_ddot=None, with_jacobian=True)
        b = fk_pin.fk_outputs(q, qd)
        e_p = max(e_p, float(np.linalg.norm(
            dq_translation(a["x"]) - dq_translation(b["x"]))))
        e_r = max(e_r, float(abs(float(
            dq_rotation(a["x"]) @ dq_rotation(b["x"])))))
        e_xi = max(e_xi, float(np.abs(a["xi"] - b["xi"]).max()))
        e_xid = max(e_xid, float(np.abs(a["xi_dot"] - b["xi_dot"]).max()))
        e_J = max(e_J, float(np.abs(a["J"] - b["J"]).max()))
        e_Jdq = max(e_Jdq, float(np.abs(
            a["Jdot_qdot"] - b["Jdot_qdot"]).max()))
    # 姿态以 1-|dot| 计（单位四元数同向点积=1）
    print(f"[1] pin FK vs B601TCPChain（{n_samples} 组随机 (q, q̇)）:")
    print(f"    位置 {e_p:.3e} m   姿态 1-|r·r'| {1.0 - e_r:.3e}")
    print(f"    xi {e_xi:.3e}   xi_dot {e_xid:.3e}   J {e_J:.3e}   "
          f"Jdot_qdot {e_Jdq:.3e}")
    # DH vs URDF 表示差 ~1e-6（verify_dh/b601_dynamics 同口径），
    # 速度/雅可比为一阶量，阈值放宽到 1e-4
    ok = (e_p < 1e-4 and (1.0 - e_r) < 1e-8 and e_xi < 1e-4
          and e_J < 1e-4 and e_Jdq < 1e-4)
    print(f"    [{'PASS' if ok else 'FAIL'}]")

    q = rng.uniform(lo, hi)
    qd = rng.uniform(-1.0, 1.0, 6)
    N = 100
    t0 = time.perf_counter()
    for _ in range(N):
        chain.fk_outputs(q, qd, q_ddot=None, with_jacobian=True)
    t_hdq = (time.perf_counter() - t0) / N * 1e3
    t0 = time.perf_counter()
    for _ in range(N):
        fk_pin.fk_outputs(q, qd)
    t_pin = (time.perf_counter() - t0) / N * 1e3
    print(f"[2] 计时: HDQ 链 {t_hdq:.3f} ms vs pin {t_pin:.3f} ms"
          f"（加速 {t_hdq / max(t_pin, 1e-9):.1f}x）")
    return ok


if __name__ == "__main__":
    _selfcheck()
