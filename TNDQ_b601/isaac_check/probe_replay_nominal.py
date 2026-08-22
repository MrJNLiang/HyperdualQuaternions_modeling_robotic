#!/usr/bin/env python3
"""名义模型开环回放 —— 与 Isaac 闭环/开环对照，二分停滞根因。

已知事实（exp1_slant_v4 + 探针）：
  1. Isaac 闭环停滞（pos_err 0.087 恒定），meas==cmd（力矩通道无误）；
  2. Isaac 开环回放同一力矩序列：前 1 s 与闭环轨迹一致，1.5 s 后大幅
     偏离（朝 SETPOINT 方向快速运动）；
  3. 纯仿真闭环（diag_loop_sim [A]）同参数收敛。

本脚本：用 B601NominalDynamics.forward_dynamics 半隐式欧拉，从 Q_INIT
开环回放 exp1 CSV 力矩序列 3 s，输出 q(t) 并与 CSV 闭环轨迹对比：
  - 名义模型回放也停滞 => 力矩序列本身（控制律）产生停滞均衡；
  - 名义模型回放快速偏离（同 Isaac 开环）=> 力矩序列驱动运动，
    Isaac 闭环停滞是执行环境（求解器/接触）问题。

运行（纯 numpy）：
    /home/liang/miniconda3/envs/dq_hinf/bin/python \
        TNDQ_b601/isaac_check/probe_replay_nominal.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.b601_dynamics import B601NominalDynamics
from config.params import DT, JOINT_DAMPING

Q_INIT = np.array([-0.251958, -2.607660, -1.617296,
                   -1.077630, 0.000041, -0.000004])


def main():
    dyn = B601NominalDynamics()
    csv = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "results", "exp1_setpoint.csv")
    with open(csv) as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(csv, delimiter=",", skiprows=1)
    ic = {c: i for i, c in enumerate(header)}
    n_rows = int(3.0 / 0.01)

    q, qd = Q_INIT.copy(), np.zeros(6)
    print("t[s]   q_replay - q_csv（闭环实测）")
    for k in range(n_rows):
        tau = np.array([data[k, ic[f"tau{i}"]] for i in range(1, 7)])
        for _ in range(5):                      # CSV 行 = 5 个物理步
            qdd = dyn.forward_dynamics(q, qd, tau)
            qd = qd + DT * qdd
            q = q + DT * qd
        if k % 50 == 0:
            q_csv = np.array([data[k, ic[f"q{i}"]] for i in range(1, 7)])
            print(f"{k*0.01:4.2f}   {np.round(q - q_csv, 4).tolist()}")
    q_csv_end = np.array([data[n_rows - 1, ic[f"q{i}"]] for i in range(1, 7)])
    print(f"3s 终点差 replay-csv = {np.round(q - q_csv_end, 4).tolist()}")


if __name__ == "__main__":
    main()
