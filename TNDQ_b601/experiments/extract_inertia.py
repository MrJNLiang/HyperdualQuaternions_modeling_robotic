"""
B601 DH 系惯性参数提取（URDF -> DH 连杆系），一次性设计工具。

流程：
  [1] XML 解析 URDF：全部 link 惯性（质量 / 质心 / 质心系 3x3 惯量）与
      关节树；
  [2] q=0 位姿：URDF link 系（base_link 系）与 DH 帧（DH 基座系 + 前缀）；
  [3] 连杆归属：DH 连杆 i <- URDF link_i；link6 并入 gripper 子树
      （gripper_link + 双手指，prismatic q=0 静态位形，手指开合对腕部
      动力学影响 ~1e-3 kg*m，作受控模型失配处理）；
  [4] 惯性参数变换到 DH 连杆系（质心 + 质心系 3x3 惯量，多刚体平行轴
      合成）；
  [5] Pinocchio 交叉对账（URDF 直接建模、锁定手指关节，rnea 全量对账
      随机 (q, v, a) 样本）。

输出：B601_LINK_MASS / B601_LINK_COM / B601_LINK_INERTIA（回填
config/b601_dynamics.py）。

用法（TNDQ_b601 目录下，需 pinocchio）：
    python3 experiments/extract_inertia.py
"""

import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.params import B601_DH_TABLE, B601_BASE_PREFIX, B601_URDF


def rpy_to_R(rpy):
    """URDF fixed-axis RPY：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_rot(axis, q):
    """Rodrigues 旋转（axis 单位向量，含负轴）。"""
    a = np.asarray(axis, dtype=float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1.0 - np.cos(q)) * (K @ K)


def _T(R, p):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def parse_urdf(path):
    """解析 URDF：links 惯性表 + 关节链（按 parent/child 拓扑排序）。"""
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        inert = link.find("inertial")
        if inert is None:
            continue
        origin = inert.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz").split()])
        rpy = np.array([float(v) for v in origin.get("rpy").split()])
        mass = float(inert.find("mass").get("value"))
        att = inert.find("inertia").attrib
        I = np.array([
            [float(att["ixx"]), float(att["ixy"]), float(att["ixz"])],
            [float(att["ixy"]), float(att["iyy"]), float(att["iyz"])],
            [float(att["ixz"]), float(att["iyz"]), float(att["izz"])]])
        links[link.get("name")] = {
            "mass": mass, "com": xyz, "R": rpy_to_R(rpy), "I": I}
    joints = []
    for j in root.findall("joint"):
        origin = j.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz").split()])
        rpy = np.array([float(v) for v in origin.get("rpy").split()])
        axis_el = j.find("axis")
        axis = (np.array([float(v) for v in axis_el.get("xyz").split()])
                if axis_el is not None else np.zeros(3))
        joints.append({
            "name": j.get("name"), "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": xyz, "rpy": rpy, "axis": axis})
    # 直接返回全部关节（树形拓扑，分支展开在 urdf_link_poses 中处理）
    return links, joints


def urdf_link_poses(joints, q_arm=0.0):
    """q_arm 位形下各 link 在 base_link 系下的位姿（含分支，固定点迭代）。

    注：非 fixed 关节统一取 q_arm（本工具只用 q=0 位形；手指 prismatic
    关节取 0 = 提取假设的静态位形）。
    """
    poses = {"base_link": np.eye(4)}
    pending = list(joints)
    while pending:
        rest = []
        for j in pending:
            if j["parent"] not in poses:
                rest.append(j)
                continue
            T = poses[j["parent"]]
            T = T @ _T(np.eye(3), j["xyz"]) @ _T(rpy_to_R(j["rpy"]), np.zeros(3))
            if j["type"] != "fixed":
                T = T @ _T(axis_rot(j["axis"], float(q_arm)), np.zeros(3))
            poses[j["child"]] = T.copy()
        if len(rest) == len(pending):
            raise ValueError("关节树含不可达分支："
                             + str([j["name"] for j in rest]))
        pending = rest
    return poses


def dh_frame_poses(q=6 * (0.0,)):
    """q=0 时各 DH 帧在 base_link（世界）系下的位姿。"""
    T = _T(np.eye(3), B601_BASE_PREFIX)
    out = []
    for i, row in enumerate(B601_DH_TABLE):
        a, alpha, d, off, _ = row
        ct, st = np.cos(q[i] + off), np.sin(q[i] + off)
        ca, sa = np.cos(alpha), np.sin(alpha)
        A = np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]])
        T = T @ A
        out.append(T.copy())
    return out


def main():
    links, joints_all = parse_urdf(B601_URDF)
    print(f"[1] URDF 解析：{len(links)} 个含惯量 link，"
          f"{len(joints_all)} 个关节（手臂 6 + 夹爪 3，树形拓扑）")

    link_poses = urdf_link_poses(joints_all)
    dh_poses = dh_frame_poses()
    print("[2] q=0 位姿计算完成（URDF link 系 + DH 帧）")

    # DH 连杆 i <- URDF link_i（link6 并入 gripper 子树，prismatic q=0）
    grip_names = ["gripper_link", "gripper_left", "gripper_right"]
    masses, coms, inertias = [], [], []
    for i in range(6):
        T_link = link_poses[f"link{i+1}"]
        T_dh = dh_poses[i]
        # DH 帧在 link_i 系下的位姿（R 把 DH 系向量转到 link 系）
        inv_link = np.eye(4)
        inv_link[:3, :3] = T_link[:3, :3].T
        inv_link[:3, 3] = -inv_link[:3, :3] @ T_link[:3, 3]
        T_link_dh = inv_link @ T_dh
        R, p = T_link_dh[:3, :3], T_link_dh[:3, 3]

        bodies = [(f"link{i+1}", np.eye(4))]
        if i == 5:
            bodies += [(nm, np.linalg.inv(T_link) @ link_poses[nm])
                       for nm in grip_names]
        m_tot, mc, I_sum = 0.0, np.zeros(3), []
        cs, ms = [], []
        for name, T_b in bodies:
            b = links[name]
            c_link = T_b[:3, :3] @ b["com"] + T_b[:3, 3]
            I_link = T_b[:3, :3] @ b["R"] @ b["I"] @ b["R"].T @ T_b[:3, :3].T
            c_dh = R.T @ (c_link - p)
            I_dh = R.T @ I_link @ R
            m_tot += b["mass"]
            mc += b["mass"] * c_dh
            cs.append(c_dh)
            ms.append(b["mass"])
            I_sum.append(I_dh)
        c = mc / m_tot
        I = np.zeros((3, 3))
        for c_k, m_k, I_k in zip(cs, ms, I_sum):
            d = c_k - c
            I += I_k + m_k * (float(d @ d) * np.eye(3) - np.outer(d, d))
        masses.append(m_tot)
        coms.append(c)
        inertias.append(I)
        extra = f"（+{len(bodies) - 1} gripper 体）" if i == 5 else ""
        print(f"[3] DH 连杆 {i + 1}{extra}: m={m_tot:.6f} kg  "
              f"c=[{c[0]:+.6f}, {c[1]:+.6f}, {c[2]:+.6f}]")

    masses, coms, inertias = np.array(masses), np.array(coms), np.array(inertias)
    print(f"    整机质量（不含 base_link）= {masses.sum():.4f} kg")

    # ---- 输出回填格式（高精度，避免截断误差污染对账阈值）-------------
    print("\n[4] 回填 config/b601_dynamics.py 的参数表：")
    print("B601_LINK_MASS = np.array(["
          + ", ".join(f"{m:.9f}" for m in masses) + "])")
    print("B601_LINK_COM = np.array([")
    for c in coms:
        print(f"    [{c[0]:+.9f}, {c[1]:+.9f}, {c[2]:+.9f}],")
    print("])")
    print("B601_LINK_INERTIA = np.array([")
    for I in inertias:
        print("    [[" + ", ".join(f"{v:+.6e}" for v in I[0]) + "],")
        print("     [" + ", ".join(f"{v:+.6e}" for v in I[1]) + "],")
        print("     [" + ", ".join(f"{v:+.6e}" for v in I[2]) + "]],")
    print("])")

    # ---- Pinocchio 交叉对账（回填 b601_dynamics.py 后重跑生效）----------------
    # 对账核心实现在 config/b601_dynamics.py::pinocchio_cross_check
    # （延迟 import pinocchio，核心模块不依赖它）；此处用提取参数而非
    # 回填表实例化，与 URDF 直接建模对账，双重保险回填数值无误。
    try:
        from config.b601_dynamics import (
            B601NominalDynamics, pinocchio_cross_check)
    except ImportError:
        print("[5] config/b601_dynamics.py 尚未创建，跳过交叉对账"
              "（用 [4] 输出回填参数表后重跑本脚本）")
        return 0
    dyn = B601NominalDynamics(mass=masses, com=coms, inertia=inertias)
    pinocchio_cross_check(dyn)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
