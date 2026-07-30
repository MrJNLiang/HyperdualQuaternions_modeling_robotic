"""
C1 (式 5.2) 增益系统性整定报告 —— 纯离线分析，不需要 CoppeliaSim。

回答“当前 K_D/K_P 是否最优”，并给出可复现的整定过程：

  阶段 0  理论核验：把标量 k_p 推广为对称正定 K_p 后，定理 3 的
          交叉项抵消 V̇ = -e_ξᵀK_d e_ξ + e_ξᵀd 是否仍精确成立
          （用非线性 A(x̃) 随机抽样数值核验，不是线性化近似）。
  阶段 1  诊断：C1 现有增益 vs C3 基线等效模型的逐通道极点/阻尼/
          直流误差增益；并用 S3 已有实测 npz 交叉验证线性预测。
  阶段 2  约束优化：在 (dt 余量, 阻尼比, H∞ 证书, 指令峰值预算) 四个
          约束下扫描主导极点，输出可行域-代价前沿与最优候选。
  阶段 3  敏感性：对 K_ω/K_v/p_O/p_T 各 ±2× 的一次一因子分析。
  阶段 4  证书：所选增益的 (5.6a)、认证 L2 增益、ISS 球、e_z 级联界。

用法（TNDQ_sim 目录下）：

    python3 experiments/tune_tndq_gains.py            # 完整报告
    python3 experiments/tune_tndq_gains.py --set tuned  # 只看某组的指标
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                ".."))

from config import params
from control import gain_design as gd
from control.error_system import A_matrix, output_error
from control.performance import (
    pose_weight, storage_function, check_hinf_condition_merged,
    tightest_certified_l2_gain, iss_ultimate_bound, ez_cascade_bound,
)

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "results")
DT = params.COPPELIA_DT_TARGET       # S3 实验控制周期（引擎步长）


# ---------------------------------------------------------------------------
# 阶段 0：矩阵 K_p 下定理 3 的耗散恒等式数值核验
# ---------------------------------------------------------------------------

def stage0_verify_matrix_kp(n_trial=200, seed=0):
    """V̇ = -e_ξᵀ K_d e_ξ + e_ξᵀ d 对任意对称正定 K_p 精确成立（非线性 A）。

    V = ½‖e_ξ‖² + ½ e_zᵀ K_p e_z，误差体系
        ė_ξ = -K_d e_ξ - Aᵀ K_p e_z + d,   ė_z = A e_ξ
    交叉项 e_zᵀK_p A e_ξ 与 -e_ξᵀAᵀK_p e_z 逐点相消（K_p 对称即可）。
    """
    rng = np.random.default_rng(seed)
    K_d = params.GAIN_SETS["tuned"]["K_d"]
    K_p = pose_weight(params.GAIN_SETS["tuned"]["k_p"])
    worst = 0.0
    for _ in range(n_trial):
        # 随机单位 DQ 误差（非近单位，检验完整非线性 A）
        r = rng.normal(size=4)
        r /= np.linalg.norm(r)
        if r[0] < 0:
            r = -r
        p = rng.normal(scale=0.3, size=3)
        # x_tilde = r + eps ½ p r
        dual = 0.5 * np.r_[
            -p @ r[1:4],
            r[0] * p + np.cross(p, r[1:4]),
        ]
        x_tilde = np.r_[r, dual]

        A = A_matrix(x_tilde)
        e_z, _, _ = output_error(x_tilde)
        e_xi = rng.normal(size=6)
        d = rng.normal(size=6)

        e_xi_dot = -K_d @ e_xi - A.T @ (K_p @ e_z) + d
        e_z_dot = A @ e_xi
        V_dot = e_xi @ e_xi_dot + e_z @ (K_p @ e_z_dot)
        claim = -e_xi @ (K_d @ e_xi) + e_xi @ d
        worst = max(worst, abs(V_dot - claim) / max(1.0, abs(claim)))

    print("阶段 0  矩阵 K_p 的定理 3 耗散恒等式核验")
    print(f"  {n_trial} 次随机抽样（完整非线性 A(x̃)、随机 e_ξ/d）")
    print(f"  max |V̇ - (-e_ξᵀK_d e_ξ + e_ξᵀd)| / |RHS| = {worst:.3e}"
          f"  -> {'精确成立（机器精度）' if worst < 1e-12 else '不成立!'}")
    print("  结论：K_p 由标量推广为对称正定矩阵后，(5.6a)/(5.6b)、认证 L2")
    print("        增益 1/λmin(K_d)、ISS 球 (5.7) 的推导逐字照搬。\n")
    return worst


# ---------------------------------------------------------------------------
# 阶段 1：诊断 + 实测交叉验证
# ---------------------------------------------------------------------------

def _fmt_channel(m):
    poles = ", ".join(f"{p:+.2f}" for p in np.sort(np.real(m["poles"]))[::-1])
    return (f"poles [{poles}]  ζ={m['zeta']:.2f}  DC={m['static_gain']:.4g}  "
            f"ts={m['t_settle']:.2f}s  |p|dt={m['pole_dt']:.3f}")


def _measured_ratio(law_tag_a, law_tag_b, key):
    """读两份 S3 npz，返回带载圆周稳态段 RMS 比 (a/b)；缺文件返回 None。"""
    vals = []
    for tag in (law_tag_a, law_tag_b):
        p = os.path.join(RESULTS_DIR, f"grasp_circle_{tag}.npz")
        if not os.path.exists(p):
            return None
        d = np.load(p, allow_pickle=False)
        t, t6 = d["t"], float(d["t_marks"][-1])
        m = t >= t6 + params.CIRCLE_RAMP_TIME + 1.0
        e = np.linalg.norm(d["e_z"][:, :3] if key == "O" else d["e_z"][:, 3:],
                           axis=1)
        vals.append(np.sqrt(np.mean(e[m] ** 2)))
    return vals[0] / vals[1]


def stage1_diagnose():
    print("阶段 1  诊断：现有 C1 增益 vs C3 基线等效模型（dt=%.0f ms）" % (DT * 1e3))
    base = gd.c1_channels(params.K_D, params.K_P, dt=DT)
    c3 = gd.c3_channels(params.DQH_GAMMA_O, params.DQH_GAMMA_T,
                        params.DQH_K_SERVO, dt=DT)
    for name in ("rotation", "translation"):
        print(f"  C1 base {name:11s} {_fmt_channel(base[name])}")
    for name in ("rotation", "translation"):
        print(f"  C3 base {name:11s} {_fmt_channel(c3[name])}")

    print("  线性预测的稳态误差比 C3/C1（常值失配扰动下 DC 增益之比）：")
    pred_O = c3["rotation"]["static_gain"] / base["rotation"]["static_gain"]
    pred_T = c3["translation"]["static_gain"] / base["translation"]["static_gain"]
    meas_O = _measured_ratio("dqhinf_load", "load", "O")
    meas_T = _measured_ratio("dqhinf_load", "load", "T")
    print(f"    |O|: 预测 {pred_O:.3f}"
          + (f"   S3 带载实测 {meas_O:.3f}" if meas_O else "   （无实测数据）"))
    print(f"    |T|: 预测 {pred_T:.3f}"
          + (f"   S3 带载实测 {meas_T:.3f}" if meas_T else "   （无实测数据）"))
    print("  判定：当前 K_D=8I / k_p=16 非最优 —— 标量 k_p 使旋转通道刚度")
    print("        仅为平移的 1/4，旋转主导极点 -0.54/s（整定时间 7.5 s，")
    print("        长于 lift/retreat/transit 各相位时长），准静态带载任务的")
    print("        稳态误差因此被直流刚度而非带宽限制。\n")
    return base, c3


# ---------------------------------------------------------------------------
# 阶段 2：约束优化（可行域 + 代价前沿）
# ---------------------------------------------------------------------------

def _reference_errors():
    """指令峰值约束用的参考误差幅值：取 S3 带载实测的 95 分位。"""
    p = os.path.join(RESULTS_DIR, "grasp_circle_load.npz")
    if not os.path.exists(p):
        return 0.05, 0.01            # 回退到 hold 段量级
    d = np.load(p, allow_pickle=False)
    e_xi = np.linalg.norm(d["e_xi"], axis=1)
    e_z = np.linalg.norm(d["e_z"], axis=1)
    return float(np.percentile(e_xi, 95)), float(np.percentile(e_z, 95))


def stage2_sweep():
    e_xi_ref, e_z_ref = _reference_errors()
    kw = dict(dt=DT, kappa=params.KAPPA, gamma_a=params.GAMMA_A,
              qddot_max=params.QDDOT_MAX, e_xi_ref=e_xi_ref, e_z_ref=e_z_ref)
    print("阶段 2  约束优化（参考误差 |e_ξ|₉₅=%.3f, |e_z|₉₅=%.4f，取自 S3 带载实测）"
          % (e_xi_ref, e_z_ref))
    print("  约束：λmin(K_d)≥%.2f (5.6a) | max|p|dt≤0.15 | ζ≥1 | 指令峰值≤%.0f rad/s²"
          % (0.5 * (1 / params.KAPPA + 1 / params.GAMMA_A ** 2), params.QDDOT_MAX))

    # 参考设计 = C3 等效（代价归一化基准）
    g_ref = gd.design_matching_c3(params.DQH_GAMMA_O, params.DQH_GAMMA_T,
                                  params.DQH_K_SERVO)
    ref = gd.screen(g_ref, **kw)

    rows = []
    # 现有组（标量 k_p）单列，其余按主导极点扫描（保持 C3 的 1:5 极点型）
    base_entry = gd.screen(dict(K_omega=8.0, K_v=8.0, p_O=16.0, p_T=16.0), **kw)
    rows.append(("base k_p=16", base_entry))
    for dom in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 12.0):
        g = gd.design_from_poles(dom, ratio=5.0)
        rows.append((f"poles -{dom:g}/-{5 * dom:g}", gd.screen(g, **kw)))
    # 标量 k_p 的最好可能（对齐平移 DC，旋转仍差 4×）——说明矩阵推广的必要性
    rows.append(("scalar k_p=80, K_d=24I",
                 gd.screen(dict(K_omega=24.0, K_v=24.0, p_O=80.0, p_T=80.0), **kw)))

    hdr = (f"  {'candidate':22s} {'K_ω':>5s} {'K_v':>5s} {'p_O':>6s} {'p_T':>6s} "
           f"{'DC_O':>8s} {'DC_T':>8s} {'ts':>6s} {'|p|dt':>6s} {'u_pk':>6s} "
           f"{'L2cert':>7s} {'feas':>10s} {'J':>6s}")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    best = None
    for name, e in rows:
        g = e["gains"]
        J = gd.cost(e, ref)
        bad = ",".join(k for k, v in e["checks"].items() if not v) or "ok"
        j_txt = f"{J:6.3f}" if np.isfinite(J) else f"{'inf':>6s}"
        print(f"  {name:22s} {g['K_omega']:5.1f} {g['K_v']:5.1f} {g['p_O']:6.0f} "
              f"{g['p_T']:6.0f} {e['static_O']:8.4g} {e['static_T']:8.4g} "
              f"{e['t_settle']:6.2f} {e['pole_dt']:6.3f} {e['u_peak']:6.1f} "
              f"{e['l2_certified']:7.3f} {bad:>10s} {j_txt}")
        if np.isfinite(J) and (best is None or J < best[1]):
            best = (name, J, e)

    print(f"  代价 J = 1.0·(DC 增益/参考)均值 + 0.5·(ts/参考) + 0.25·(u_pk/QDDOT_MAX)，")
    print(f"  参考 = C3 等效设计（J=1 即与基线同预算）。最优可行候选：{best[0]}（J={best[1]:.3f}）")
    print("  选定 tuned 组 = 与 C3 逐通道恒等的同预算设计点：")
    print(f"    K_ω=K_v={g_ref['K_omega']:.0f}, p_O={g_ref['p_O']:.0f}, p_T={g_ref['p_T']:.0f}"
          f"  (J={gd.cost(ref, ref):.3f})")
    print("  取舍（如实）：J 最小的可行点是 -6/-30（params GAIN_SETS['fast']），")
    print("    但它恰好落在 |p|dt=0.15 的显式积分余量边界上、指令峰值预算也翻倍；")
    print("    -8/-40 起则 disc+eff 双越界。因此：")
    print("      · tuned（-4/-20，与 C3 逐通道恒等）用于公平对比 —— 剩余差异只")
    print("        反映结构而非调参；")
    print("      · fast（-6/-30）同样入仿真验证，检验成本函数的外推是否可信")
    print("        （饱和步数/治理次数/附着冲击是线性模型看不到的代价）。\n")
    return g_ref, ref, kw


# ---------------------------------------------------------------------------
# 阶段 3：敏感性
# ---------------------------------------------------------------------------

def stage3_sensitivity(g, ref, kw):
    print("阶段 3  参数敏感性（一次一因子，×0.5 / ×2）")
    print(f"  {'param':8s} {'factor':>7s} {'DC_O':>9s} {'DC_T':>9s} {'ζ_O':>6s} "
          f"{'ζ_T':>6s} {'ts':>6s} {'|p|dt':>6s} {'u_pk':>6s} {'J':>7s} {'feas':>10s}")
    for key, f, e in gd.sensitivity(g, **kw):
        J = gd.cost(e, ref)
        bad = ",".join(k for k, v in e["checks"].items() if not v) or "ok"
        print(f"  {key:8s} {f:7.1f} {e['static_O']:9.4g} {e['static_T']:9.4g} "
              f"{e['channels']['rotation']['zeta']:6.2f} "
              f"{e['channels']['translation']['zeta']:6.2f} {e['t_settle']:6.2f} "
              f"{e['pole_dt']:6.3f} {e['u_peak']:6.1f} "
              f"{J if np.isfinite(J) else float('inf'):7.3f} {bad:>10s}")
    print("  读法：p_O/p_T（刚度）直接决定稳态 DC 增益，减半即误差翻倍；")
    print("        K_ω/K_v（阻尼）不改 DC 增益，只改 ζ 与整定时间——减半会")
    print("        使 ζ<1 产生过冲（接触任务禁忌），加倍会把慢极点拖回")
    print("        主导地位（ts 变长）。刚度加倍则 |p|dt 与指令峰值同时逼近约束。\n")


# ---------------------------------------------------------------------------
# 阶段 4：证书
# ---------------------------------------------------------------------------

def stage4_certificates():
    print("阶段 4  H∞/ISS 证书（各增益组）")
    d_inf = 1.0        # 单位扰动下的可比口径
    print(f"  {'set':8s} {'λmin(K_d)':>10s} {'(5.6a)需':>9s} {'L2认证':>8s} "
          f"{'ISS球/‖d‖':>10s} {'e_z级联界':>10s} {'V权重':>18s}")
    for name, gs in params.GAIN_SETS.items():
        K_d, k_p = gs["K_d"], gs["k_p"]
        ok, lam, level = check_hinf_condition_merged(K_d, params.KAPPA,
                                                     params.GAMMA_A)
        l2 = tightest_certified_l2_gain(K_d)
        iss = iss_ultimate_bound(K_d, d_inf)
        ez = ez_cascade_bound(iss, K_d, k_p)
        Kp = pose_weight(k_p)
        w = f"diag({Kp[0, 0]:.0f}·I₃, {Kp[3, 3]:.0f}·I₃)"
        print(f"  {name:8s} {lam:10.2f} {level:9.2f} {l2:8.3f} {iss:10.4f} "
              f"{ez:10.4f} {w:>18s}"
              + ("" if ok else "   [(5.6a) 不满足]"))
    print("  注：V 的权重随 K_p 变化，跨组比较 V 时须用同一参考权重重算")
    print("      （run_grasp_circle.py 的对比表已按 base 权重统一折算）。\n")


# ---------------------------------------------------------------------------
# 阶段 5：整定策略有效性验证（线性预测 vs S3 实测）
# ---------------------------------------------------------------------------

def _ss_rms(tag):
    """读 S3 npz，返回圆周稳态段的 (|O|rms, |T|rms)；缺文件返回 None。"""
    p = os.path.join(RESULTS_DIR, f"grasp_circle_{tag}.npz")
    if not os.path.exists(p):
        return None
    d = np.load(p, allow_pickle=False)
    m = d["t"] >= float(d["t_marks"][-1]) + params.CIRCLE_RAMP_TIME + 1.0
    out = []
    for sl in (slice(0, 3), slice(3, 6)):
        e = np.linalg.norm(d["e_z"][:, sl], axis=1)
        out.append(float(np.sqrt(np.mean(e[m] ** 2))))
    return tuple(out)


def stage5_validate():
    """用带载 S3 实测检验“直流刚度决定准静态稳态误差”的整定依据。

    对每个增益组，预测的稳态误差比（相对 C3）= 两者 DC 增益之比；
    实测比 = 圆周稳态段 RMS 之比。两者一致则整定模型可用于外推。"""
    print("阶段 5  整定策略有效性验证（带载 S3 圆周稳态段，基准 = C3）")
    ref = _ss_rms("dqhinf_load")
    if ref is None:
        print("  （缺 C3 带载实测，跳过）\n")
        return
    c3 = gd.c3_channels(params.DQH_GAMMA_O, params.DQH_GAMMA_T,
                        params.DQH_K_SERVO, dt=DT)
    print(f"  C3 实测基准：|O|rms={ref[0]:.3e}, |T|rms={ref[1]:.3e}")
    print(f"  {'set':8s} {'|O| 预测/C3':>12s} {'|O| 实测/C3':>12s} "
          f"{'|T| 预测/C3':>12s} {'|T| 实测/C3':>12s}")
    for name, gs in params.GAIN_SETS.items():
        tag = "load" if name == "base" else f"{name}_load"
        meas = _ss_rms(tag)
        ch = gd.c1_channels(gs["K_d"], gs["k_p"], dt=DT)
        pO = ch["rotation"]["static_gain"] / c3["rotation"]["static_gain"]
        pT = ch["translation"]["static_gain"] / c3["translation"]["static_gain"]
        if meas is None:
            print(f"  {name:8s} {pO:12.3f} {'未跑':>12s} {pT:12.3f} {'未跑':>12s}")
            continue
        print(f"  {name:8s} {pO:12.3f} {meas[0] / ref[0]:12.3f} "
              f"{pT:12.3f} {meas[1] / ref[1]:12.3f}")
    print("  判定：预测与实测同向且同阶即证实整定依据（DC 刚度）成立；")
    print("        tuned 组预测比为 1.000（逐通道恒等），实测应同样贴近 1。\n")


def show_set(name):
    gs = params.GAIN_SETS[name]
    print(f"增益组 {name}: K_d=diag({np.diag(gs['K_d'])[0]:.0f}I₃,"
          f"{np.diag(gs['K_d'])[3]:.0f}I₃), "
          f"K_p=diag({np.diag(pose_weight(gs['k_p']))[0]:.0f}I₃,"
          f"{np.diag(pose_weight(gs['k_p']))[3]:.0f}I₃)")
    for ch, m in gd.c1_channels(gs["K_d"], gs["k_p"], dt=DT).items():
        print(f"  {ch:11s} {_fmt_channel(m)}")


def main():
    ap = argparse.ArgumentParser(description="C1 增益系统性整定报告（离线）")
    ap.add_argument("--set", choices=list(params.GAIN_SETS),
                    help="只打印某个增益组的通道指标")
    args = ap.parse_args()

    if args.set:
        show_set(args.set)
        return

    print("=" * 78)
    print("C1 (式 5.2) 增益整定报告 —— control/gain_design.py")
    print("=" * 78)
    stage0_verify_matrix_kp()
    stage1_diagnose()
    g, ref, kw = stage2_sweep()
    stage3_sensitivity(g, ref, kw)
    stage4_certificates()
    stage5_validate()
    print("实验复现（同一实验环境下验证）：")
    print("  python3 experiments/run_grasp_circle.py --gains tuned --mode noload")
    print("  python3 experiments/run_grasp_circle.py --gains tuned --mode load")
    print("  python3 experiments/run_grasp_circle.py --gains fast  --mode load")
    print("  python3 experiments/run_grasp_circle.py --compare-only --plot")
    print("=" * 78)


if __name__ == "__main__":
    main()
