"""
Tabular data logger -- records the time histories required by the paper's
experiment protocol (Sec. 6.3) and prints/saves them as plain tables.
No plotting anywhere (per project requirement).

Logged columns per sample:
    t                     time stamp
    e_z = [O; T]          pose error, rotation part O and translation part T
                          (Theorem 1 / output map (4.4))
    e_xi                  geometrically consistent twist error (Theorem 1, (4.3))
    qddot_ref             control command of formula (5.2)
    x_d, xi_d, xi_dot_d   desired trajectory (TNDQ chain, (3.3a)/(3.5))
    x, xi                 actual pose/twist from the measured TNDQ chain
    V                     storage function 1/2||e_xi||^2 + k_p/2||e_z||^2 (Sec. 5.3)
    d_hat                 reconstructed certificate-channel disturbance
                          (control.performance.ResidualDisturbanceEstimator;
                          diagnostic only, never enters the control law)
    c0, c1, c2            unit-constraint residual family (3.8)
    runtime               per-step controller wall time [s]

Outputs:
    - .npz archive with all raw arrays
    - fixed-width text table (summary columns) for quick inspection
    - printed summary of key metrics (final errors, RMS, max residuals,
      measured vs certified H-infinity gain if a PerformanceAccumulator
      summary is supplied)
"""

import os
import numpy as np

from core.dq_algebra import dq_translation, dq_rotation


class DataLogger:
    """Accumulates per-step records and writes table/npz/csv outputs."""

    FIELDS = ["t", "e_z", "e_xi", "qddot_ref", "tau",
              "x_d", "xi_d", "xi_dot_d", "x", "xi",
              "V", "d_hat", "c0", "c1", "c2", "runtime"]

    def __init__(self):
        self._rows = {name: [] for name in self.FIELDS}

    def log(self, t, e_z, e_xi, qddot_ref, x_d, xi_d, xi_dot_d, x, xi,
            V, c0, c1, c2, runtime, tau=None, d_hat=None):
        r = self._rows
        r["t"].append(float(t))
        r["e_z"].append(np.asarray(e_z, dtype=float))
        r["e_xi"].append(np.asarray(e_xi, dtype=float))
        r["qddot_ref"].append(np.asarray(qddot_ref, dtype=float))
        # 力矩模式（τ = M̂ q̈_ref + Ĉ q̇ + ĝ，§2.4）时记录实际下发力矩；
        # 加速度级理想仿真（式 5.1）无 τ，补零保持列结构一致
        if tau is None:
            tau = np.zeros_like(np.asarray(qddot_ref, dtype=float))
        r["tau"].append(np.asarray(tau, dtype=float))
        r["x_d"].append(np.asarray(x_d, dtype=float))
        r["xi_d"].append(np.asarray(xi_d, dtype=float))
        r["xi_dot_d"].append(np.asarray(xi_dot_d, dtype=float))
        r["x"].append(np.asarray(x, dtype=float))
        r["xi"].append(np.asarray(xi, dtype=float))
        r["V"].append(float(V))
        # 反演的证书通道等效扰动 d̂（诚实条款：含 ΔM/Δg、噪声、伪逆残差、
        # 限幅/治理器与离散化；不进控制律）。未提供时补零保持列结构
        r["d_hat"].append(np.zeros(6) if d_hat is None
                          else np.asarray(d_hat, dtype=float))
        r["c0"].append(float(c0))
        r["c1"].append(float(c1))
        r["c2"].append(float(c2))
        r["runtime"].append(float(runtime))

    def as_arrays(self):
        return {k: np.asarray(v) for k, v in self._rows.items()}

    # -- persistence ---------------------------------------------------------

    def save_npz(self, path, extra=None):
        """Raw arrays -> .npz (mirrors the repository's result convention)."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        data = self.as_arrays()
        if extra:
            data.update({k: np.asarray(v) for k, v in extra.items()})
        np.savez(path, **data)
        return path

    def save_table(self, path, every=1):
        """
        Fixed-width text table (no plots).  Norm columns keep the table
        readable; full vectors live in the .npz archive.
        """
        os.makedirs(os.path.dirname(path), exist_ok=True)
        d = self.as_arrays()
        header = (f"{'t':>8} {'|O|':>11} {'|T|':>11} {'|e_xi|':>11} "
                  f"{'|qddot_ref|':>12} {'V':>11} "
                  f"{'c0':>10} {'c1':>10} {'c2':>10} {'runtime':>10}")
        lines = [header, "-" * len(header)]
        for k in range(0, len(d["t"]), every):
            O_n = np.linalg.norm(d["e_z"][k][:3])
            T_n = np.linalg.norm(d["e_z"][k][3:])
            lines.append(
                f"{d['t'][k]:8.3f} {O_n:11.3e} {T_n:11.3e} "
                f"{np.linalg.norm(d['e_xi'][k]):11.3e} "
                f"{np.linalg.norm(d['qddot_ref'][k]):12.3e} "
                f"{d['V'][k]:11.3e} "
                f"{d['c0'][k]:10.2e} {d['c1'][k]:10.2e} {d['c2'][k]:10.2e} "
                f"{d['runtime'][k]:10.2e}")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        return path

    def save_csv(self, path):
        """
        CSV 输出（替代定宽 txt 表，便于 pandas/电子表格后处理）。

        列定义（总方案 §5.4 指标 A/B/D）：
            t                  时间戳 [s]
            pos_err            位置误差范数 ‖p - p_d‖ [m]（由 DQ 平移部还原）
            ori_err_geodesic   姿态测地距离 θ = 2 arccos(|<r, r_d>|) [rad]
            e_xi_norm          几何一致 twist 误差范数（定理 1，式 4.3）
            e_z_O_norm/T_norm  输出误差旋转/平移分量范数（定理 1，式 4.4）
            qddot_ref_norm     控制指令范数（式 5.2）
            tau_norm           下发力矩范数 ‖τ‖ [N m]（§2.4；加速度级为 0）
            V                  存储函数（定理 3 Lyapunov 监控）
            d_hat_norm         反演的证书通道等效扰动范数 ‖d̂‖（§6.5(6)：由
                               闭环误差动态 (5.1e) 反演，含 ΔM/Δg、噪声、
                               伪逆残差、限幅/治理器与离散化；仅诊断量）
            c0,c1,c2           约束族 (3.8) 残差（数值健康度，判据 < 1e-12）
            runtime            控制器单步壁钟 [s]（实时性指标 B）
        """
        os.makedirs(os.path.dirname(path), exist_ok=True)
        d = self.as_arrays()
        header = ("t,pos_err,ori_err_geodesic,e_xi_norm,e_z_O_norm,e_z_T_norm,"
                  "qddot_ref_norm,tau_norm,V,d_hat_norm,c0,c1,c2,runtime")
        lines = [header]
        for k in range(len(d["t"])):
            # 位置误差：从实际/期望 DQ 还原平移 p = 2 q_d' r*（式 2.1 逆映射）
            p = dq_translation(d["x"][k])
            p_d = dq_translation(d["x_d"][k])
            pos_err = np.linalg.norm(p - p_d)
            # 姿态误差：单位四元数测地距离（双覆盖下取 |内积|，
            # 与定理 1(i) 符号翻转不变性一致）
            r = dq_rotation(d["x"][k])
            r_d = dq_rotation(d["x_d"][k])
            ori_err = 2.0 * np.arccos(min(1.0, abs(float(r @ r_d))))
            lines.append(
                f"{d['t'][k]:.6f},{pos_err:.9e},{ori_err:.9e},"
                f"{np.linalg.norm(d['e_xi'][k]):.9e},"
                f"{np.linalg.norm(d['e_z'][k][:3]):.9e},"
                f"{np.linalg.norm(d['e_z'][k][3:]):.9e},"
                f"{np.linalg.norm(d['qddot_ref'][k]):.9e},"
                f"{np.linalg.norm(d['tau'][k]):.9e},"
                f"{d['V'][k]:.9e},"
                f"{np.linalg.norm(d['d_hat'][k]):.9e},"
                f"{d['c0'][k]:.6e},{d['c1'][k]:.6e},{d['c2'][k]:.6e},"
                f"{d['runtime'][k]:.6e}")
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        return path

    # -- numerical summary (no plots) -----------------------------------------

    def print_summary(self, performance_summary=None, tail_fraction=0.2):
        """
        Key numerical indicators: final/RMS errors, control effort,
        constraint-residual maxima (3.8), runtime statistics, and the
        Theorem 3 performance certificates if provided.
        """
        d = self.as_arrays()
        n_tail = max(1, int(tail_fraction * len(d["t"])))
        O = np.linalg.norm(d["e_z"][:, :3], axis=1)
        T = np.linalg.norm(d["e_z"][:, 3:], axis=1)
        exi = np.linalg.norm(d["e_xi"], axis=1)
        u = np.linalg.norm(d["qddot_ref"], axis=1)

        print("=" * 68)
        print("Numerical summary (paper Sec. 6.3 metrics)")
        print("=" * 68)
        print(f"  samples                     : {len(d['t'])}   "
              f"t in [{d['t'][0]:.3f}, {d['t'][-1]:.3f}] s")
        print(f"  final |O|, |T|              : {O[-1]:.3e}, {T[-1]:.3e}")
        print(f"  steady |O| (mean tail)      : {np.mean(O[-n_tail:]):.3e}")
        print(f"  steady |T| (mean tail)      : {np.mean(T[-n_tail:]):.3e}")
        print(f"  steady |e_xi| (mean tail)   : {np.mean(exi[-n_tail:]):.3e}")
        print(f"  RMS |e_xi| / max |e_xi|     : "
              f"{np.sqrt(np.mean(exi ** 2)):.3e} / {np.max(exi):.3e}")
        print(f"  RMS |qddot_ref| / max       : "
              f"{np.sqrt(np.mean(u ** 2)):.3e} / {np.max(u):.3e}")
        print(f"  V(0) -> V(end)              : {d['V'][0]:.3e} -> {d['V'][-1]:.3e}")
        print(f"  max c0, c1, c2  (3.8)       : "
              f"{np.max(d['c0']):.2e}, {np.max(d['c1']):.2e}, {np.max(d['c2']):.2e}")
        print(f"  controller runtime mean/max : "
              f"{np.mean(d['runtime']):.2e} / {np.max(d['runtime']):.2e} s")

        if performance_summary is not None:
            s = performance_summary
            print("-" * 68)
            print("Theorem 3 performance certificates")
            print(f"  H-inf inequality (5.6): LHS {s['hinf_lhs_5_6']:.4e}  "
                  f"<=  RHS {s['hinf_rhs_5_6']:.4e}  "
                  f"[{'OK' if s['hinf_lhs_5_6'] <= s['hinf_rhs_5_6'] else 'VIOLATED'}]")
            print(f"  measured L2 gain            : {s['measured_l2_gain']:.4e}")
            print(f"  certified L2 gain (remark)  : {s['certified_l2_gain']:.4e}")
            print(f"  per-channel gains (5.6')    : omega {s['measured_gain_omega']:.3e}, "
                  f"v {s['measured_gain_v']:.3e}")
            # (5.7) 是均方（RMS）界，不是逐点 ISS 极限球 —— 与 e_xi 的 RMS
            # 同口径，故可直接比；d̂ = 反演的全部扰动源，d_inj = 注入的 J w
            print(f"  RMS bound (5.7) on |e_xi|   : {s['iss_bound_e_xi']:.4e} "
                  f">= measured RMS {s['e_xi_rms']:.4e}  "
                  f"[margin x{s['rms_margin']:.1f}]")
            print(f"  ||d_hat||_inf (reconstructed): {s['d_inf']:.3e}   "
                  f"||d_inj||_inf (J w only) : {s['d_inj_inf']:.3e}")
            if s['d_inj_inf'] > 0.0:
                print(f"  measured L2 gain (inj. only): "
                      f"{s['measured_l2_gain_injected']:.4e}")
        print("=" * 68)
