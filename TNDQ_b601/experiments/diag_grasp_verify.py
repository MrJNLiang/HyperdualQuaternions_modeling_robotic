#!/usr/bin/env python3
"""抓取位物理对账：复现抓取构型，读物理手指/方块位姿，判断方块是否在指间。

exp1 v10：FK 显示方块在 gripper 局部 [~11,-0.3,-0.7]mm（开合向居中），
但物理手指闭合穿过方块（未夹住）。本诊断 teleport 抓取构型 + 方块就位，
直接读 gripper_link / gripper_left / gripper_right / cube 的世界位姿，
全部变换到 gripper_link 系，输出方块与两指的位置关系，并慢闭合看接触。
运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_grasp_verify.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config.params as P
import experiments.ik_lib as IK
from experiments.ik_lib import solve_ik, world_to_dh
IK.IK_LO, IK.IK_HI = P.JOINT_LOWER.copy(), P.JOINT_UPPER.copy()


def _R_from_quat(r):
    w, x, y, z = r
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1 - 2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1 - 2*(x*x+y*y)]])


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True)
    backend.setup()
    # pxr / RigidPrim 必须在 SimulationApp 启动后 import（Isaac 铁律）
    from pxr import Usd, UsdGeom
    from isaacsim.core.prims import RigidPrim
    stage = backend.stage

    # 抓取 IK（与 run_lib 目标一致：SETPOINT_POS + R_TOOL_QUAT）
    q0 = np.array([-0.3, -1.6, -0.6, -0.6, 0.0, 0.0])
    q, res, ok = solve_ik(world_to_dh(P.SETPOINT_POS), P.R_TOOL_QUAT, q0)
    assert ok, f"IK 失败 {res}"
    print("grasp q =", np.round(q, 4))

    def link_pose(rel):
        prim = stage.GetPrimAtPath(P.ISAAC_ROBOT_PRIM + rel)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        gq = xf.ExtractRotation().GetQuaternion()
        qq = np.array([gq.GetReal(), *gq.GetImaginary()], dtype=float)
        return np.array(xf.ExtractTranslation(), dtype=float), qq/np.linalg.norm(qq)

    def physx_pose(rel):
        """PhysX 地面真值：RigidPrim.get_world_pose（区别于 USD xform）。"""
        rp = RigidPrim(prim_paths_expr=P.ISAAC_ROBOT_PRIM + rel,
                       name=rel.split("/")[-1] + "_rp", reset_xform_properties=False)
        pos, quat = rp.get_world_poses()
        return (np.asarray(pos, dtype=float).reshape(-1)[0:3],
                np.asarray(quat, dtype=float).reshape(-1)[0:4])

    # teleport 抓取位 + 张开 + 方块就位
    backend.reset_to(q, gripper_width=P.GRIPPER_OPENING)
    c0, cq0 = P.CUBE_POS.copy(), None
    cube0, cq0 = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0, cq0)
    for _ in range(30):
        backend.articulation.set_joint_positions(
            backend._row(q), joint_indices=backend.arm_idx)
        backend.articulation.set_joint_positions(
            backend._row(np.full(2, 0.5*P.GRIPPER_OPENING)),
            joint_indices=backend.grip_idx)
        backend.articulation.set_joint_velocities(
            backend._row(np.zeros(8)), joint_indices=None)
        backend.step()

    p_g, q_g = backend.get_ee_pose()
    p_gx, q_gx = physx_pose(backend._GRIPPER_LINK_REL)
    print("\nUSD xform vs PhysX 真值（gripper_link 世界位姿）:")
    print("  USD  p =", np.round(p_g, 5))
    print("  PhysX p =", np.round(p_gx, 5), " 偏差 =",
          f"{np.linalg.norm(p_gx - p_g)*1e3:.2f} mm")
    print("  quat 点积 =", f"{abs(np.dot(q_g, q_gx)):.6f}")
    Rg = _R_from_quat(q_g)
    def to_gripper(p_w):
        return Rg.T @ (np.asarray(p_w) - p_g)

    pl, ql = link_pose(backend._FINGER_RELS[0])
    pr, qr = link_pose(backend._FINGER_RELS[1])
    plx, _ = physx_pose(backend._FINGER_RELS[0])
    prx, _ = physx_pose(backend._FINGER_RELS[1])
    print("左指原点 USD/PhysX 偏差 =",
          f"{np.linalg.norm(plx-pl)*1e3:.2f} mm；右指 =",
          f"{np.linalg.norm(prx-pr)*1e3:.2f} mm")
    cc, _ = backend.get_cube_pose()

    print("\n== gripper_link 系（x=接近/下, y=开合, z） ==")
    print("gripper_link 世界 p =", np.round(p_g, 4))
    print("方块中心 局部 =", np.round(to_gripper(cc)*1e3, 1), "mm")
    print("左指原点 局部 =", np.round(to_gripper(pl)*1e3, 1), "mm")
    print("右指原点 局部 =", np.round(to_gripper(pr)*1e3, 1), "mm")
    # 方块 8 角点在 gripper 系，看开合向(y)范围与接近向(x)范围
    hsz = P.CUBE_SIZE/2
    ycaw, xcaw = np.cos(cube0[2]*0+P.CUBE_YAW), 0  # 用 CUBE_YAW
    cy = P.CUBE_YAW
    Rcz = np.array([[np.cos(cy), -np.sin(cy), 0],
                    [np.sin(cy),  np.cos(cy), 0], [0, 0, 1]])
    corners_w = np.array([cc + Rcz @ np.array([sx*hsz, sy*hsz, sz*hsz])
                          for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)])
    cg = np.array([to_gripper(c) for c in corners_w])
    print("方块角点 gripper 系范围:")
    for ax, nm in ((0,"接近x"),(1,"开合y"),(2,"z")):
        print(f"   {nm}: [{cg[:,ax].min()*1e3:+7.1f}, {cg[:,ax].max()*1e3:+7.1f}] mm")
    print("指垫区 x_ee∈[-20,0]mm；两指原点的开合向 y 即各自行程侧。")

    # 慢闭合：用真实驱动通道 set_gripper（位置 drive）逐档收开度，
    # 读方块与实测开度，检测首次扰动。注意：旧版用 set_joint_positions
    # teleport 手指与 reset_to 设置的 drive 目标（张开位）互相打架，
    # 实测 w 恒滞后指令 ~10 mm，数据无效；必须走 drive 通道。
    # ★v12b 语义锤定：真实开度 = 2*stroke（stroke=0 并拢、
    # 0.0715 全开）；旧版“首触 stall w≈93 mm”实为开度 93 mm 处
    # 扰动（比垫面预期 45 mm 早得多，需定位是哪个结构接触）。
    print("\n== 慢闭合（drive stroke 细扫 70 -> 42 mm），PhysX 直测真实开度 ==")
    base_xy = cc[:2].copy()
    for stroke_cmd in [0.070, 0.065, 0.060, 0.058, 0.056, 0.054, 0.052,
                       0.050, 0.048, 0.046, 0.044, 0.042]:
        for _ in range(500):
            backend.articulation.set_joint_positions(
                backend._row(q), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.set_gripper(2.0 * stroke_cmd)
            backend.step()
        cnow, _ = backend.get_cube_pose()
        w_meas = backend.get_gripper_width()         # = 2*stroke 实测
        plx, _ = physx_pose(backend._FINGER_RELS[0])
        prx, _ = physx_pose(backend._FINGER_RELS[1])
        real_gap = np.linalg.norm(to_gripper(plx) - to_gripper(prx))
        pg_now, _ = physx_pose(backend._GRIPPER_LINK_REL)
        arm_drift = np.linalg.norm(pg_now - p_gx) * 1e3
        dvec = cnow[:2]-base_xy
        print(f"  stroke={stroke_cmd*1e3:4.1f}mm 真实开度={real_gap*1e3:5.1f}mm "
              f"臂漂移={arm_drift:4.1f}mm "
              f"方块移位=({dvec[0]*1e3:+5.2f},{dvec[1]*1e3:+5.2f})mm "
              f"cz={cnow[2]*1e3:6.1f}mm")
        if np.linalg.norm(dvec) > 0.02:   # 首触后停止（避免推飞）
            print("  -> 首次扰动超 20 mm，停止细扫")
            break
    backend.close()


if __name__ == "__main__":
    main()
