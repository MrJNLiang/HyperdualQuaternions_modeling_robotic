"""
TNDQ_real/scripts/01_calib_signs.py —— 关节符号/零位标定（交互式）。

标定坐标系（右手系，＝URDF 基座系；观察者站照片位）：
  +z 竖直向上；+y 水平朝观察者右手（肘侧）；+x 深度方向背离观察者
  （零位时沿伸直臂指向夹爪）。

阶段：
  A. 电机系重力保持（厂商动力学 + 逐关节自适应标度，臂稳定可掰动）；
  B. 逐关节手掰定符号：脚本以当前实测姿态经 Jacobian 打印该关节
     URDF 正方向的指尖速度方向（坐标轴表述），操作者用手把关节
     掰到该方向，脚本读编码器位移符号自动定 JOINT_SIGN；
  C. 手动掰臂到 Q_INIT 姿态 -> 抓零位 OFFSET；
  D. FK 输出当前 TCP 位置供卷尺复核，打印 transforms.py 回填块。

安全：全程重力保持托臂；Ctrl+C 立即失能。符号的最终裁决在 02 的
URDF 系重力保持：符号错则保持立即发散（2 s 短窗观察即可发现）。

运行：python scripts/01_calib_signs.py（conda rebot，工作目录 = TNDQ_real/）
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config.paths  # noqa: F401,E402

from config.params import (B601_BASE_PREFIX, B601_DH_TABLE,  # noqa: E402
                           B601_TOOL_ANGLE, Q_INIT)

# 标定坐标系（右手系，观察者站照片位：夹爪朝左、肘朝右）：
#   +z 竖直向上；+x 水平指向肘侧（观察者右手）；+y = z×x 水平背离观察者。
# URDF 正方向＝右手定则绕下表轴；括号内为折臂姿态下指尖/部件速度方向。
# 运行时 describe() 按实测姿态打印活方向，以打印为准。
URDF_POS_DIR = [
    "j1: 右手绕 +z 轴（俯视逆时针）",
    "j2: 右手绕 -y 轴（指尖 +z 上升）",
    "j3: 右手绕 +y 轴（折臂态指尖 +z 上升）",
    "j4: 右手绕 +y 轴（紫色支架点头；腕簇 +z 上升）",
    "j5: 右手绕 -z 轴（竖直腕偏航，电机 0x05；俯视顺时针为正）",
    "j6: 右手绕 +x 轴（黑筒滚转=夹爪旋转，电机 0x06；"
    "从夹爪端朝基座看逆时针为正）",
]

# 逐关节脉冲幅值/时长已废弃（改为手掰法，无主动激励）


def main():
    from reBotArm_control_py.actuator import RebotArm
    from reBotArm_control_py.dynamics import compute_generalized_gravity

    robot = RebotArm()
    robot.connect()
    raw = np.zeros(6)                   # 最新 raw 关节角（电机系）
    scale = np.array([1.0, 1.55, 1.55, 1.55, 1.0, 1.0])  # 逐关节重力标度初值
    v_lp = np.zeros(6)
    st = {"q": None, "t": time.perf_counter()}
    # raw->模型系 暂定符号：j3/j4 与 URDF 反（照片观测）；j5/j6 顺接
    # （0x05=joint5 偏航、0x06=joint6 滚转，零位 pinocchio 实算 +
    # Studio 点动确认）。重力必须用模型系姿态算 g(S*q)，且下发力矩
    # 必须乘同一符号（tau = S*g，见 ctrl）——历次"保持反了"的根因
    # 就是漏乘了这个 S。阶段 B 每确定一个符号会实时回写。
    SIGN_HOLD = np.array([1.0, 1.0, -1.0, -1.0, 1.0, 1.0])

    from reBotArm_control_py.dynamics import load_dynamics_model
    import pinocchio as pin
    _m = load_dynamics_model()
    _d = _m.createData()

    def describe(i, sign_cur):
        """以当前实测姿态算 URDF 正方向在用户视角的物理描述。"""
        s = np.where(np.arange(6) < i, sign_cur, 1.0)
        qm = np.concatenate([s * raw, [0.0, 0.0]])
        pin.forwardKinematics(_m, _d, qm)
        pin.updateFramePlacements(_m, _d)
        pin.computeJointJacobians(_m, _d, qm)
        jid = _m.getJointId(f"joint{i + 1}")
        ax = _d.J[3:6, _m.joints[jid].idx_v]
        p6 = _d.oMi[6].translation
        p_ref = p6 + _d.oMi[6].rotation @ np.array([0.1, 0.0, 0.0])  # 指尖点
        v = np.cross(ax, p_ref - _d.oMi[jid].translation)
        hb = p_ref - _d.oMi[1].translation
        hb = np.array([hb[0], hb[1], 0.0]); hb /= max(np.linalg.norm(hb), 1e-9)
        e_up = np.array([0.0, 0.0, 1.0])
        e_out = np.cross(-hb, e_up)          # 朝观察者
        c = np.array([v @ hb, v @ e_up, v @ e_out])
        # hb=水平指向指尖(照片左)=-y；e_up=+z；e_out=朝观察者=-x
        words = [("-y（水平左，夹爪侧）", "+y（水平右，肘侧）"),
                 ("+z（竖直上）", "-z（竖直下）"),
                 ("-x（深度，朝观察者）", "+x（深度，背离观察者）")]
        k = int(np.argmax(np.abs(c)))
        return words[k][0] if c[k] > 0 else words[k][1]

    def ctrl(r, dt):
        q = r.arm.get_positions(request_feedback=True)
        now = time.perf_counter()
        if st["q"] is not None:           # 差分速度 + 低通
            v = (q[:6] - st["q"]) / max(now - st["t"], 1e-4)
            v_lp[:] = 0.9 * v_lp + 0.1 * v
        st["q"], st["t"] = q[:6].copy(), now
        raw[:] = q[:6]
        g = compute_generalized_gravity(q=SIGN_HOLD * q[:6])[:6]  # 模型系姿态
        # 在线自适应：模型系速度 v_m = SIGN_HOLD*v_lp；持续下坠 -> scale 增大
        mask = np.abs(v_lp) < 0.8
        scale[:] = np.clip(scale - 0.3 * g * (SIGN_HOLD * v_lp) * mask * dt,
                           0.5, 4.0)
        tau = SIGN_HOLD * (g * scale)   # 力矩逆变换：电机系 = SIGN*模型系
        r.arm.send_mit(pos=q[:6], vel=np.zeros(6),
                       kp=np.ones(6), kd=np.full(6, 1.0), tau=tau)

    robot.arm.mode_mit()
    robot.enable_all()
    robot.start_control_loop(ctrl, rate=200)
    print("[01] 重力保持已启动：臂应稳定悬停（厂商模型，电机系）")

    sign = np.ones(6)
    try:
        # ---- 阶段 B：逐关节手掰定符号 ----
        print("\n[01-B] 标定坐标系（右手系）：+z 竖直上；+y 水平朝你右手"
              "（肘侧）；+x 深度方向背离你（零位时指向夹爪）。")
        print("[01-B] 方法：按打印方向用手掰对应关节，脚本读编码器定符号；"
              "重力保持下可随时掰动。")
        for i in range(6):
            word = describe(i, sign)
            while True:
                q_before = raw.copy()
                input(f"\n[01-B] j{i + 1}: 用手掰该关节使指尖「{word}」"
                      f"（只动 j{i + 1}，另一手托住臂其余部分），"
                      f"掰到位保持住，回车记录...")
                dq = raw[i] - q_before[i]
                if abs(dq) >= 0.05:
                    break
                print(f"        位移 {dq:+.4f} rad 太小，掰大一点再记录")
            sign[i] = 1.0 if dq > 0 else -1.0
            SIGN_HOLD[i] = sign[i]      # 回写重力保持
            print(f"[01-B] j{i + 1}: raw 位移 {dq:+.4f} rad -> sign={sign[i]:+.0f}"
                  f"（raw 与 URDF {'同向' if sign[i] > 0 else '反向'}）")

        # ---- 阶段 C：掰臂到 Q_INIT 抓零位 ----
        print(f"\n[01-C] 现在请手动把臂掰到 Q_INIT 姿态："
              f"{np.round(Q_INIT, 3).tolist()} rad")
        print("       （近似零位：j2/j3 各向内 -0.10 rad≈-5.7°，其余 0；"
              "保持重力托住即可掰动）")
        input("       掰到位后回车抓取零位...")
        offset = Q_INIT - sign * raw.copy()
        q_now = sign * raw + offset
        print(f"[01-C] OFFSET 抓取完成：当前 URDF 系 q="
              f"{np.round(q_now, 4).tolist()}（应≈Q_INIT）")

        # ---- 阶段 D：FK 复核（卷尺测 gripper 原点 = 指尖端）----
        from core.dq_algebra import dq_mul, dq_rot_z, dq_translation
        from core.kinematics import TNDQSerialChain

        chain = TNDQSerialChain(B601_DH_TABLE)
        x = dq_mul(chain.fkm(q_now), dq_rot_z(B601_TOOL_ANGLE))
        p = np.asarray(dq_translation(x), float) + B601_BASE_PREFIX
        print(f"[01-D] FK 预测 gripper 原点（base_link 系）="
              f"{np.round(p, 4).tolist()} m")
        print("       请用卷尺复核实测指尖端位置；偏差 > 1 cm 说明符号"
              "或零位有误（FK 呈镜像 = 某关节符号反）")

        # ---- 回填块 ----
        print(f"\n[01] 自适应重力标度 scale={np.round(scale, 3).tolist()}")
        print("========== 回填 TNDQ_real/config/transforms.py ==========")
        print(f"JOINT_SIGN = np.array({sign.tolist()})")
        print(f"JOINT_OFFSET = np.array({np.round(offset, 6).tolist()})")
        print("=========================================================")
        print("下一步：回填后运行 scripts/02_gravity_hold.py 做 URDF 系"
              "重力保持（符号终审 + TAU_SCALE 整定）")
    finally:
        robot.stop_control_loop()
        robot.disable_all()
        robot.disconnect()
        print("[01] 已失能断连")


if __name__ == "__main__":
    main()
