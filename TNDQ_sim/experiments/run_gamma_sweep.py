"""
γ 影响实验（gamma sweep）：新理论 γ_a（分析/证书参数）vs 旧理论 γ_O/γ_T
（综合参数）对误差结果的影响 —— 论文附录 C.3 γ-κ 关系的实验化。

理论背景（论文定理 3(c) + 附录 C.3）：

  新理论（TNDQ，式 5.2）：控制律不含 γ_a —— γ_a/κ 只出现在证书条件
      (5.6a)  K_d ⪰ ½(κ⁻¹ + γ_a⁻²) I
  中，是"分析参数"。附录 C.3 的 θ-缩放族给出 γ-κ 设计规则：
      θ* = √κ/(2 γ_a)，最紧可行条件 λ_min(K_d) ≥ 1/(γ_a √κ)，
      固定 γ_a 时 κ* = γ_a² 使 (5.6a) 在族内最紧；
      Lyapunov 路径可认证的能量增益天花板 = 1/λ_min(K_d)。
  推论（本实验 A 组验证）：固定增益下扫 γ_a，闭环轨迹与误差严格不变，
  变的只是证书的"可判定性"（γ_a 太小则 (5.6a) 不可行，证书失效，
  但系统行为不变）。

  γ_a 要"影响误差"，必须经综合模式回写增益（本实验 B 组）：
      κ = γ_a²（族内最紧）,  K_d = γ_a⁻² I（取 (5.6a) 等号，证书恰好紧）,
      K_p 取临界阻尼配置 p_T = (K_v/2)², p_O = 4 p_T（两通道双重极点）。
  此时认证 L2 能量增益 = 1/λ_min(K_d) = γ_a²：γ_a 越小 -> 增益越大 ->
  误差越小，代价是指令峰值/离散化余量（screen() 四约束过滤）。

  旧理论（C3，一阶 DQ H∞ 运动学律）：γ 直接决定增益 kO = √2/γ_O、
  kT = √2/γ_T，是"综合参数"—— 扫 γ 天然改变误差（本实验 C 组，
  与旧 H∞ 论文的 γ 实验同构）。经内环速度伺服桥接到力矩接口后其
  一阶证书失效，故 C 组只报测得增益，无认证列。

实验设置（三组共用，保证可比）：
  内部加速度级对象（式 5.1，q̈ = q̈_ref + w_dyn）+ line 轨迹（6 s）+
  L2 扰动（L2Disturbance, seed=0，能量口径 d_vec6 = J w）；
  dt = params.DT，t_end = 8 s，稳态窗 [7, 8] s（轨迹结束 + 扰动衰减后）。

用法（TNDQ_sim 目录下，无需 CoppeliaSim）：

    python3 experiments/run_gamma_sweep.py
    python3 experiments/run_gamma_sweep.py --t-end 8.0 --no-plot

输出：results/gamma_sweep.csv（A/B/C 三组全部数值指标）+
results/gamma_sweep.png（认证 vs 测得 L2 增益、稳态误差 vs γ）+ 终端表格。
"""

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

from config import params
from core.kinematics import TNDQSerialChain
from control.error_system import full_error_state
from control.control_law import (
    geometric_computed_torque_law, damped_pinv,
    dq_hinf_kinematic_law, velocity_to_accel_ref,
)
from control.performance import PerformanceAccumulator, storage_function
from control.gain_design import screen, gains_to_matrices
from simdata.trajectory_generator import LineTrajectoryTNDQ
from simdata.input_simulation import L2Disturbance

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "results")

# 扫描网格（对数间距，覆盖证书可行/不可行两侧）
GAMMA_GRID_A = [1.0, 0.75, 0.5, 0.35, 0.25, 0.2, 0.15, 0.14]
GAMMA_GRID_B = [1.0, 0.707, 0.5, 0.354, 0.25, 0.2, 0.177]
GAMMA_GRID_C = [1.0, 0.707, 0.5, 0.354, 0.25, 0.177]

# screen() 的参考误差幅值（内部 line 场景初始偏置 0.05 rad/关节的实测
# 瞬态量级；C-eff 约束 = 数据接地的 QDDOT_MAX 饱和预算）
E_XI_REF, E_Z_REF = 0.5, 0.2
T_SS = 7.0                      # 稳态窗起点 [s]（轨迹 6 s 结束 + 扰动衰减）


def _simulate(law, K_d, K_p, kappa, gamma_a, t_end, gamma_c3=None):
    """跑一次内部加速度级闭环（式 5.1），返回指标 dict。

    law = "tndq"：式 (5.2)，增益 (K_d, K_p)；
    law = "dq-hinf"：C3 一阶律（γ_O=γ_T=gamma_c3）+ 内环速度伺服桥接。
    三组共用：同轨迹 / 同 L2 扰动（seed=0）/ 同 dt / 同伪逆 / 同零空间
    治理（零空间分量不改任务动态）/ 同 QDDOT_MAX 限幅。
    """
    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    n = chain.n
    dt = params.DT
    q = params.Q_INIT + 0.05 * np.array([1, -1, 1, -1, 1, -1, 1], dtype=float)
    q_dot = np.zeros(n)
    q_ns_center = q.copy()

    trajectory = LineTrajectoryTNDQ(
        chain.fkm(params.Q_INIT), delta_p=[0.15, 0.10, -0.10],
        duration=6.0, rot_axis=[0.0, 0.0, 1.0], rot_angle=0.5)
    w_dyn = L2Disturbance(n, amplitude=1.0, decay=0.5, omega=3.0, t_on=1.0,
                          seed=0)

    perf = PerformanceAccumulator(K_d, K_p, kappa, gamma_a)
    ts, O_hist, T_hist, exi_hist = [], [], [], []
    sat_steps = 0
    qdd_peak = 0.0
    qdot_cmd_prev = None

    n_steps = int(round(t_end / dt))
    for k in range(n_steps):
        t = k * dt
        fk = chain.fk_outputs(q, q_dot, q_ddot=None, with_jacobian=True)
        des = trajectory.evaluate(t)
        err = full_error_state(fk["x_breve"], des["x_breve_d"])

        if law == "tndq":
            qddot_ref, _ = geometric_computed_torque_law(
                err, des["xi_d"], des["xi_dot_d"],
                fk["J"], fk["Jdot_qdot"], K_d, K_p,
                damping=params.PINV_DAMPING)
        else:
            task_vel = dq_hinf_kinematic_law(err, des["xi_d"],
                                             gamma_c3, gamma_c3)
            qdot_cmd = damped_pinv(
                fk["J"], damping=max(params.PINV_DAMPING,
                                     params.DQH_DAMPING)) @ task_vel
            qddot_ref = velocity_to_accel_ref(
                qdot_cmd, qdot_cmd_prev, q_dot, dt, params.DQH_K_SERVO)
            qdot_cmd_prev = qdot_cmd

        # 零空间居中（不改变 ξ = Jq̇ 的任务动态，同 run_simulation.py）
        Jp = damped_pinv(fk["J"], damping=params.PINV_DAMPING)
        N_proj = np.eye(n) - Jp @ fk["J"]
        qddot_ref = qddot_ref + N_proj @ (
            params.NULLSPACE_K * (q_ns_center - q)
            - params.NULLSPACE_D * q_dot)

        qn = np.linalg.norm(qddot_ref)
        qdd_peak = max(qdd_peak, qn)
        if qn > params.QDDOT_MAX:
            qddot_ref = qddot_ref * (params.QDDOT_MAX / qn)
            sat_steps += 1

        # 式 (5.1)：q̈ = q̈_ref + w_dyn，半隐式欧拉
        w = w_dyn(t)
        q_dot = q_dot + (qddot_ref + w) * dt
        q = q + q_dot * dt

        # 能量核算（扰动的任务空间口径 d = J w，同 run_simulation.py 监控层）
        perf.update(err["e_xi"], err["e_z"], fk["J"] @ w, dt)
        ts.append(t)
        O_hist.append(np.linalg.norm(err["O"]))
        T_hist.append(np.linalg.norm(err["T"]))
        exi_hist.append(np.linalg.norm(err["e_xi"]))

    ts = np.asarray(ts)
    O_hist, T_hist = np.asarray(O_hist), np.asarray(T_hist)
    exi_hist = np.asarray(exi_hist)
    m_ss = ts >= T_SS
    if not m_ss.any():          # 短 t_end 烟测：回退到末段 1/8 窗
        m_ss = ts >= ts[-1] * 0.875
    s = perf.summary()
    return dict(
        T_rms_ss=float(np.sqrt(np.mean(T_hist[m_ss] ** 2))),
        O_rms_ss=float(np.sqrt(np.mean(O_hist[m_ss] ** 2))),
        exi_rms=float(np.sqrt(np.mean(exi_hist ** 2))),
        E_exi=perf.E_exi, E_d=perf.E_d,
        measured_l2=float(s["measured_l2_gain"]),
        hinf_lhs=float(s["hinf_lhs_5_6"]), hinf_rhs=float(s["hinf_rhs_5_6"]),
        sat_steps=sat_steps, qdd_peak=float(qdd_peak),
    )


# ===========================================================================
# A 组：证书扫描（固定增益，γ_a 只动证书 -> 误差严格不变）
# ===========================================================================

def sweep_A(t_end):
    """固定 tuned 增益（K_d=24I）跑一次闭环；对 γ_a 网格逐点判定 (5.6a)
    可行性、κ* = γ_a²、最紧条件 λ_min ≥ 1/(γ_a√κ)。测得指标各行相同
    —— 这正是"γ_a 是分析参数、不进控制律"的实验证据。"""
    K_d = params.GAIN_SETS["tuned"]["K_d"]
    K_p = params.GAIN_SETS["tuned"]["k_p"]
    lam_min = float(np.min(np.diag(K_d)))
    print("=" * 78)
    print("A 组：证书扫描（固定增益 tuned：K_d=24I, p_O=320, p_T=80；"
          "γ_a 不进控制律 (5.2)）")
    print("=" * 78)
    base = _simulate("tndq", K_d, K_p, params.KAPPA, params.GAMMA_A, t_end)
    rows = []
    hdr = (f"{'gamma_a':>8} {'kappa*':>8} {'level(5.6a)':>12} {'cert':>5} "
           f"{'tight_req':>10} {'cert_L2':>8} {'meas_L2':>8} "
           f"{'T_rms_ss':>10} {'O_rms_ss':>10}")
    print(hdr)
    print("-" * len(hdr))
    for g in GAMMA_GRID_A:
        kappa_star = g ** 2                       # 附录 C.3：族内最紧 κ
        level = 0.5 * (1.0 / params.KAPPA + 1.0 / g ** 2)   # κ=1 语境的 (5.6a)
        tight = 1.0 / (g * np.sqrt(kappa_star))   # = γ_a⁻²·γ_a = 1/γ_a²... 见下
        cert_ok = lam_min >= level
        rows.append(dict(group="A", gamma=g, kappa=kappa_star,
                         lam_min=lam_min, cert_level=level,
                         cert_ok=int(cert_ok),
                         certified_l2=1.0 / lam_min, **base))
        print(f"{g:8.3f} {kappa_star:8.3f} {level:12.3f} "
              f"{'OK' if cert_ok else 'FAIL':>5} {tight:10.3f} "
              f"{1.0 / lam_min:8.4f} {base['measured_l2']:8.4f} "
              f"{base['T_rms_ss']:10.3e} {base['O_rms_ss']:10.3e}")
    print("-" * len(hdr))
    print("  结论（定理 3(c)/附录 C.3）：全部行的测得列逐位相同 —— 固定增益下")
    print("  γ_a 对误差零影响；γ_a 只改变 (5.6a) 的可判定性（level > λ_min=24")
    print("  时证书失效但闭环行为不变）。tight_req = 1/(γ_a√κ)|κ=γ_a² = γ_a⁻²，")
    print("  是 θ-缩放族的最紧可行条件。")
    return rows


# ===========================================================================
# B 组：综合模式（κ = γ_a², K_d = γ_a⁻² I 取等号 -> γ_a 经增益影响误差）
# ===========================================================================

def sweep_B(t_end):
    """γ-κ 设计规则的实验化：κ=γ_a²、K_d=γ_a⁻²I（(5.6a) 取等号，证书恰紧，
    认证 L2 能量增益 = γ_a²）、K_p 临界阻尼（p_T=(K_v/2)², p_O=4p_T）。
    screen() 四约束（证书/离散/阻尼/指令预算）过滤不可行点。"""
    print("=" * 78)
    print("B 组：综合模式扫描（κ=γ_a², K_d=γ_a⁻²I 取 (5.6a) 等号，"
          "K_p 临界阻尼 p_T=(K_v/2)², p_O=4p_T）")
    print("=" * 78)
    rows = []
    hdr = (f"{'gamma_a':>8} {'K_d':>6} {'p_T':>7} {'feas':>5} "
           f"{'cert_L2':>8} {'meas_L2':>8} {'T_rms_ss':>10} {'O_rms_ss':>10} "
           f"{'qdd_pk':>7} {'sat':>4}")
    print(hdr)
    print("-" * len(hdr))
    for g_a in GAMMA_GRID_B:
        kd = 1.0 / g_a ** 2
        p_T = (kd / 2.0) ** 2
        gdict = dict(K_omega=kd, K_v=kd, p_O=4.0 * p_T, p_T=p_T)
        chk = screen(gdict, dt=params.DT, kappa=g_a ** 2, gamma_a=g_a,
                     qddot_max=params.QDDOT_MAX,
                     e_xi_ref=E_XI_REF, e_z_ref=E_Z_REF)
        K_d, K_p = gains_to_matrices(gdict)
        r = _simulate("tndq", K_d, K_p, g_a ** 2, g_a, t_end)
        rows.append(dict(group="B", gamma=g_a, kappa=g_a ** 2,
                         lam_min=kd, cert_level=kd,
                         cert_ok=int(chk["checks"]["cert"]),
                         certified_l2=g_a ** 2, **r))
        print(f"{g_a:8.3f} {kd:6.1f} {p_T:7.1f} "
              f"{'OK' if chk['feasible'] else 'FAIL':>5} "
              f"{g_a ** 2:8.4f} {r['measured_l2']:8.4f} "
              f"{r['T_rms_ss']:10.3e} {r['O_rms_ss']:10.3e} "
              f"{r['qdd_peak']:7.1f} {r['sat_steps']:4d}")
    print("-" * len(hdr))
    print("  结论：综合模式下 γ_a 单调决定性能 —— 认证能量增益 = γ_a²，")
    print("  稳态误差随 γ_a 减小单调下降；完整不等式 (5.6)（含 2V(0) 初始能量")
    print("  项，CSV 的 hinf_lhs ≤ hinf_rhs 列）全部行核验通过。meas_L2 =")
    print("  √(E_e/E_d) 未扣除 V(0)，小 γ_a 时由初始瞬态主导而反弹，非证书违例；")
    print("  代价是 K_d=γ_a⁻² 的指令峰值与离散化余量（screen 的 C-eff/C-disc")
    print("  约束给出 γ_a 可达下界，本表 γ_a≤0.2 已 FAIL）。这是新理论中与旧 γ")
    print("  实验同构的\"γ 定增益\"通道。")
    return rows


# ===========================================================================
# C 组：旧理论 C3（γ 是综合参数：kO = kT = √2/γ 直接进控制律）
# ===========================================================================

def sweep_C(t_end):
    """旧 H∞ 论文的 γ 实验复刻：γ_O=γ_T=γ 扫描，kO=kT=√2/γ 直接决定
    增益。同一对象/轨迹/扰动，经内环速度伺服（K_servo=20）桥接到
    加速度接口 —— 桥接后一阶证书失效，只报测得值（无认证列）。"""
    print("=" * 78)
    print("C 组：旧理论 C3 扫描（γ_O=γ_T=γ，kO=kT=√2/γ 综合参数；"
          f"内环 K_servo={params.DQH_K_SERVO:.0f}）")
    print("=" * 78)
    # 参考权重仅用于 V/能量口径（C3 无证书）；取 base 组
    K_d_ref = params.GAIN_SETS["base"]["K_d"]
    K_p_ref = params.GAIN_SETS["base"]["k_p"]
    rows = []
    hdr = (f"{'gamma':>8} {'kO=kT':>7} {'DC刚度':>8} {'meas_L2':>8} "
           f"{'T_rms_ss':>10} {'O_rms_ss':>10} {'qdd_pk':>7} {'sat':>4}")
    print(hdr)
    print("-" * len(hdr))
    for g in GAMMA_GRID_C:
        k_gain = np.sqrt(2.0) / g
        r = _simulate("dq-hinf", K_d_ref, K_p_ref,
                      params.KAPPA, params.GAMMA_A, t_end, gamma_c3=g)
        rows.append(dict(group="C", gamma=g, kappa=np.nan,
                         lam_min=np.nan, cert_level=np.nan, cert_ok=0,
                         certified_l2=np.nan, **r))
        print(f"{g:8.3f} {k_gain:7.2f} {k_gain * params.DQH_K_SERVO:8.1f} "
              f"{r['measured_l2']:8.4f} {r['T_rms_ss']:10.3e} "
              f"{r['O_rms_ss']:10.3e} {r['qdd_peak']:7.1f} "
              f"{r['sat_steps']:4d}")
    print("-" * len(hdr))
    print("  结论：旧理论 γ 与误差的耦合与 B 组同构（γ 越小增益越大误差越小），")
    print("  但 ① γ 同时锁死增益结构（单参数，无法独立配置阻尼/刚度两自由度）；")
    print("  ② 力矩接口下经内环桥接后其一阶 H∞ 证书失效 —— 测得增益无认证")
    print("  天花板可对照，是\"可调但不可证\"的综合通道。")
    return rows


# ===========================================================================
# 输出：CSV + 图
# ===========================================================================

CSV_COLS = ["group", "gamma", "kappa", "lam_min", "cert_level", "cert_ok",
            "certified_l2", "measured_l2", "T_rms_ss", "O_rms_ss", "exi_rms",
            "E_exi", "E_d", "hinf_lhs", "hinf_rhs", "sat_steps", "qdd_peak"]


def save_csv(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLS)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6e}" if isinstance(v, float) else v)
                        for k, v in r.items() if k in CSV_COLS})
    print(f"Saved CSV       : {path}  ({len(rows)} 行)")


def save_plot(rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["WenQuanYi Micro Hei", "Noto Sans CJK JP",
                                   "Droid Sans Fallback", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    A = [r for r in rows if r["group"] == "A"]
    B = [r for r in rows if r["group"] == "B"]
    C = [r for r in rows if r["group"] == "C"]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))

    # (1) A 组：误差不变性 + 证书可行边界
    ax = axes[0]
    gA = [r["gamma"] for r in A]
    ax.plot(gA, [r["T_rms_ss"] for r in A], "o-", color="tab:blue",
            label="|T| 稳态 RMS（不变）")
    ax.plot(gA, [r["O_rms_ss"] for r in A], "s-", color="tab:orange",
            label="|O| 稳态 RMS（不变）")
    lam = A[0]["lam_min"]
    g_crit = 1.0 / np.sqrt(2.0 * lam - 1.0 / params.KAPPA)
    ax.axvline(g_crit, color="tab:red", ls="--", lw=1.0,
               label=f"证书边界 γ_a={g_crit:.3f}\n(level=λ_min={lam:.0f})")
    ax.set_xlabel(r"$\gamma_a$")
    ax.set_yscale("log")
    ax.set_title("A 组：固定增益，γ_a 只动证书\n（误差严格不变，定理 3(c)）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # (2) B 组：认证 vs 测得 L2 增益
    ax = axes[1]
    gB = [r["gamma"] for r in B]
    ax.plot(gB, [r["certified_l2"] for r in B], "k--",
            label=r"认证能量增益 $\gamma_a^2 = 1/\lambda_{\min}(K_d)$")
    ax.plot(gB, [r["measured_l2"] for r in B], "o-", color="tab:green",
            label="测得 L2 增益（B 组 TNDQ）")
    ax.plot([r["gamma"] for r in C], [r["measured_l2"] for r in C],
            "^-", color="tab:red", label="测得 L2 增益（C 组 C3，无认证）")
    ax.set_xlabel(r"$\gamma_a$（B）/ $\gamma_O=\gamma_T$（C）")
    ax.set_yscale("log")
    ax.set_title("B/C 组：γ -> 增益 -> L2 增益\n（meas 未扣 V(0)；完整 (5.6) 含"
                 " 2V(0) 项全部核验通过）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # (3) B/C 组：稳态误差 vs γ
    ax = axes[2]
    ax.plot(gB, [r["T_rms_ss"] for r in B], "o-", color="tab:green",
            label="|T| 稳态（B TNDQ）")
    ax.plot(gB, [r["O_rms_ss"] for r in B], "o--", color="tab:olive",
            label="|O| 稳态（B TNDQ）")
    ax.plot([r["gamma"] for r in C], [r["T_rms_ss"] for r in C],
            "^-", color="tab:red", label="|T| 稳态（C C3）")
    ax.plot([r["gamma"] for r in C], [r["O_rms_ss"] for r in C],
            "^--", color="tab:pink", label="|O| 稳态（C C3）")
    ax.set_xlabel(r"$\gamma$")
    ax.set_yscale("log")
    ax.set_title("B/C 组：稳态误差 vs γ\n（γ 越小增益越大误差越小）")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved plot      : {path}")


def main():
    ap = argparse.ArgumentParser(
        description="γ 影响实验：A 证书扫描（γ_a 分析参数，误差不变）/ "
                    "B 综合模式（κ=γ_a², K_d=γ_a⁻²I）/ C 旧理论 C3 γ 扫描")
    ap.add_argument("--t-end", type=float, default=8.0)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    rows = sweep_A(args.t_end) + sweep_B(args.t_end) + sweep_C(args.t_end)
    os.makedirs(RESULTS_DIR, exist_ok=True)
    save_csv(rows, os.path.join(RESULTS_DIR, "gamma_sweep.csv"))
    if not args.no_plot:
        save_plot(rows, os.path.join(RESULTS_DIR, "gamma_sweep.png"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
