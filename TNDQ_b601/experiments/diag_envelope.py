#!/usr/bin/env python3
"""夹爪接触包络物理测绘（零几何假设）。

臂 kinematic 硬设保持良定构型，夹爪分别置于闭合(s=0)/张开(s=0.07)；
探针微立方体（1 cm，5 g）逐点 teleport 到 gripper 系网格，6 步后读
位移：位移 > 阈值 = 该点被夹爪几何占据。输出 ASCII 切片图（z=0 平面，
x 列 y 行）与关键尺寸（伸展范围、指垫内面、通道宽度）。

运行：~/isaacsim/python.sh TNDQ_b601/experiments/diag_envelope.py
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.params import ISAAC_ROBOT_PRIM


def _quat_to_R(r):
    w, x, y, z = r
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def main():
    from interfaces.isaac_interface import IsaacB601Backend
    backend = IsaacB601Backend(headless=True, probe_size=0.01)
    backend.setup()

    # 抓取方块挪走，避免干扰
    cube0, cq0 = backend.get_cube_pose()
    backend.cube.set_world_pose(cube0 + np.array([2.0, 0, 0]), cq0)

    q_hold = np.array([0.0, -0.6, -0.6, 0.0, 0.0, 0.0])
    xs = np.arange(-0.16, 0.0201, 0.006)
    ys = np.arange(-0.10, 0.1001, 0.006)
    zs = np.arange(-0.04, 0.0401, 0.01)
    THR = 0.0035

    def kin_hold(s, n=3):
        for _ in range(n):
            backend.articulation.set_joint_positions(
                backend._row(q_hold), joint_indices=backend.arm_idx)
            backend.articulation.set_joint_positions(
                backend._row(np.full(2, s)), joint_indices=backend.grip_idx)
            backend.articulation.set_joint_velocities(
                backend._row(np.zeros(8)), joint_indices=None)
            backend.step()

    for s, tag in ((0.07, "张开 s=0.07"), (0.0, "闭合 s=0")):
        kin_hold(s, 20)
        p_g, q_g = backend.get_ee_pose()
        Rg = _quat_to_R(q_g)
        occ = np.zeros((len(xs), len(ys), len(zs)), dtype=bool)
        total = len(xs) * len(ys) * len(zs)
        k = 0
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                for l, z in enumerate(zs):
                    k += 1
                    pt = p_g + Rg @ np.array([x, y, z])
                    backend.probe.set_world_pose(
                        pt, np.array([1.0, 0, 0, 0]))
                    backend.probe.set_linear_velocity(np.zeros(3))
                    backend.probe.set_angular_velocity(np.zeros(3))
                    for _ in range(6):
                        backend.articulation.set_joint_positions(
                            backend._row(q_hold), joint_indices=backend.arm_idx)
                        backend.articulation.set_joint_positions(
                            backend._row(np.full(2, s)),
                            joint_indices=backend.grip_idx)
                        backend.articulation.set_joint_velocities(
                            backend._row(np.zeros(8)), joint_indices=None)
                        backend.step()
                    pp, _ = backend.get_probe_pose()
                    occ[i, j, l] = np.linalg.norm(pp - pt) > THR
            if k % (len(xs) * len(ys)) == 0 or i % 5 == 4:
                print(f"  [{tag}] x={x:.3f} 进度 {100*i//len(xs)}%", flush=True)
        # ---- 输出 ----
        print(f"== {tag}：占据点数 {occ.sum()}/{total} ==")
        # z=0 切片 ASCII（x 列，y 行）
        lz = int(np.argmin(np.abs(zs)))
        sl = occ[:, :, lz]
        yline = "".join(f"{y:+.2f} "[:1] for y in ys[::8])
        print(f"   z={zs[lz]:+.2f} 切片（行=y 从 {ys[0]:+.2f} 到 {ys[-1]:+.2f}，"
              f"列=x 从 {xs[0]:+.2f} 到 {xs[-1]:+.2f}）")
        for j in range(len(ys) - 1, -1, -2):
            print("   " + "".join("#" if sl[i, j] else "."
                                  for i in range(len(xs))))
        # 数值摘要
        anyz = occ.any(axis=2)
        if anyz.any():
            ii, jj = np.where(anyz)
            print(f"   x 范围 [{xs[ii.min()]:.3f},{xs[ii.max()]:.3f}]  "
                  f"y 范围 [{ys[jj.min()]:.3f},{ys[jj.max()]:.3f}]")
            for i in range(0, len(xs), 3):
                col = anyz[i]
                if col.any():
                    jlo, jhi = np.where(col)[0][[0, -1]]
                    print(f"   x={xs[i]:+.3f}: y占据 [{ys[jlo]:+.3f},{ys[jhi]:+.3f}]"
                          f"  通道宽={max(0.0, 0):.3f}")
        # z 向覆盖（中心列）
        ic = int(np.argmin(np.abs(xs + 0.05)))
        zcol = occ[ic, len(ys) // 2, :]
        if zcol.any():
            lzlo, lzhi = np.where(zcol)[0][[0, -1]]
            print(f"   x={xs[ic]:+.3f},y=0 处 z 占据 [{zs[lzlo]:+.3f},{zs[lzhi]:+.3f}]")
    backend.close()


if __name__ == "__main__":
    main()
