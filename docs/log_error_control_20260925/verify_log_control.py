#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数值验证：对数误差控制律（GPT 建议方案）的关键数学断言
========================================================
按论文 texdocs/sections 的约定实现：
  单位 DQ: x̂ = r̂ + ε (1/2) p r̂                     (式 2.1)
  空间 twist: ξ = 2 ẋ̂ x̂* = ω + ε v,  v = ṗ + p×ω    (式 2.2)
  误差: x̃ = x̂ x̂_d*,  ξ̃ = 2 ẋ̃ x̃*,  e_z = [O; T],
        O = -Im r̃, T = p̃                            (式 4.3/4.5)
  A(x̃) = [[-1/2(η̃I + [O]×), 0], [-[T]×, I]]         (式 4.5)
  ad_a b = 1/2 (ab - ba)                             (第 2 章)

验证项：
  A. DQ 对数 exp/log 往返（2 ln x̃ = φ + ερ, ρ = V(φ)^{-1} t）
  B. dexp^{-1} 恒等式: d/dt(2 ln x̃) = ψ(ad_z) ξ̃,  ψ(s)=s/(e^s-1)（Bernoulli 级数截断）
  C. A_log 块结构（旋转块 = Jl^{-1}(φ)，右上块为零）与近恒等极限 A_log→I
  D. 反馈幅值: C1 旋转反馈 = k|sinθ|（K_{p,O}=4k）vs 对数 = kθ（沿轴不变性 Jr^{-1}φ=φ）；
     6D 情形"精确整形 - 裸对数"(即 Chandra 式(35) 形式)之差的量级
  E. 闭环误差动力学仿真：等局部刚度（DC 刚度 80）、等阻尼（极点 {-4,-20}）下
     C1（论文式 5.2 反馈）vs 对数控制器 vs 对数+误差相关阻尼，θ0 = 3.0 rad
  F. 力矩最优映射（min ‖M q̈ + h‖² s.t. J q̈ = b）的 KKT 验证与 J^+ 映射对比
仅依赖 numpy。
"""
import numpy as np

rng = np.random.default_rng(20260925)
np.set_printoptions(precision=4, suppress=True)

# ----------------------------------------------------------------------
# 四元数 (w,x,y,z) 与 DQ (r, d) 工具
# ----------------------------------------------------------------------
def pure(v):
    """3 向量 → 纯四元数"""
    v = np.asarray(v, dtype=float)
    return np.concatenate([[0.0], v])

def vec3(q):
    return q[1:]

def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2])

def qconj(a):
    return a * np.array([1.0, -1.0, -1.0, -1.0])

def qexp_pure(w):                    # 纯四元数指数
    th = np.linalg.norm(vec3(w))
    if th < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0]) + 0.5*w
    v = vec3(w)
    return np.concatenate([[np.cos(th)], np.sin(th)/th * v])

def dq_from_rt(r, t3):               # 式(2.1): x̂ = r + ε (1/2) t r
    return r, 0.5*qmul(pure(t3), r)

def dq_mul(x, y):
    r1, d1 = x; r2, d2 = y
    return qmul(r1, r2), (qmul(r1, d2) + qmul(d1, r2))

def dq_translation(x):               # t = 2 d r*，返回 3 向量
    r, d = x
    return vec3(2*qmul(d, qconj(r)))

def ad_dq(z, xi):                    # ad_z ξ = 1/2 (zξ - ξz)，z、xi 为纯 DQ (r,d)
    r1, d1 = dq_mul(z, xi)
    r2, d2 = dq_mul(xi, z)
    return (0.5*(r1 - r2), 0.5*(d1 - d2))

def bracket(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

# ----------------------------------------------------------------------
# SE(3)/DQ 对数与指数闭式
# ----------------------------------------------------------------------
def V_mat(phi):                      # V(φ) = I + (1-cosθ)/θ² [φ]× + (θ-sinθ)/θ³ [φ]×²
    th = np.linalg.norm(phi)
    P = bracket(phi)
    if th < 1e-8:
        return np.eye(3) + 0.5*P + (1.0/6)*P@P
    return np.eye(3) + (1-np.cos(th))/th**2 * P + (th-np.sin(th))/th**3 * (P@P)

def Vinv_mat(phi):                   # V^{-1} = I - 1/2[φ]× + (1-(θ/2)cot(θ/2))/θ² [φ]×²
    th = np.linalg.norm(phi)
    P = bracket(phi)
    if th < 1e-8:
        return np.eye(3) - 0.5*P + (1.0/12)*P@P
    c = (1 - (th/2)/np.tan(th/2)) / th**2
    return np.eye(3) - 0.5*P + c*(P@P)

def dq_log(x):                       # 2 ln x̃ → [φ; ρ] ∈ R^6
    r, d = x
    eta = r[0]; mu = vec3(r)
    s = np.linalg.norm(mu)
    theta = 2*np.arctan2(s, eta)
    phi = theta * mu / s if s > 1e-12 else np.zeros(3)
    t = dq_translation(x)
    rho = Vinv_mat(phi) @ t
    return np.concatenate([phi, rho])

def dq_exp_z(z6):                    # (φ, ρ) → 单位 DQ（dq_log 的逆）
    phi = z6[:3]; rho = z6[3:]
    r = qexp_pure(pure(0.5*phi))
    return dq_from_rt(r, V_mat(phi) @ rho)

def dq_exp_xi(xi6):               # SE(3) 指数 Exp(传入的 6 维量)（ξ 为 6 维空间 twist 的缩放）
    return dq_exp_z(xi6)

def paper_A(x):                      # 论文式 (4.5)
    r, d = x
    eta = r[0]; O = -vec3(r)
    T = dq_translation(x)
    A = np.zeros((6, 6))
    A[:3, :3] = -0.5*(eta*np.eye(3) + bracket(O))
    A[3:, :3] = -bracket(T)
    A[3:, 3:] = np.eye(3)
    return A

def ez_paper(x):                     # 论文 e_z = [O; T]
    return np.concatenate([-vec3(x[0]), dq_translation(x)])

# ----------------------------------------------------------------------
# A_log = ψ(ad_z) 的 6×6 矩阵（把 Bernoulli 算子打到 6 个基向量上）
#   ψ(s) = s/(e^s-1) = 1 - s/2 + s²/12 - s⁴/720 + s⁶/30240 - ...
# ----------------------------------------------------------------------
def bernoulli_numbers(N):
    """B_0..B_N，B_1 = -1/2 约定（母函数 s/(e^s-1)）
    递推: B_m = -1/(m+1) Σ_{j=0}^{m-1} C(m+1,j) B_j"""
    B = np.zeros(N+1); B[0] = 1.0
    from math import comb
    for m in range(1, N+1):
        B[m] = -sum(comb(m+1, k)*B[k] for k in range(m))/(m+1)
    return B

BERN = bernoulli_numbers(30)          # 收敛比 θ/2π < 1/2，30 项对 θ<π 达机器精度

def ad_map(z6):
    """ad_z: ξ=(ω,v) ↦ (φ×ω, φ×v − ω×ρ)，z=(φ,ρ)（与 DQ 括号 ½(zξ−ξz) 逐分量一致）"""
    phi, rho = z6[:3], z6[3:]
    def L(xi6):
        w, v = xi6[:3], xi6[3:]
        return np.concatenate([np.cross(phi, w), np.cross(phi, v) - np.cross(w, rho)])
    return L

def ad_map_T(z6):
    """ad_z 的转置（伴随表示）: (ω,v) ↦ (−φ×ω − ρ×v, −φ×v)"""
    phi, rho = z6[:3], z6[3:]
    def L(xi6):
        w, v = xi6[:3], xi6[3:]
        return np.concatenate([-np.cross(phi, w) - np.cross(rho, v), -np.cross(phi, v)])
    return L

def psi_apply(z6, xi6, adjoint=False, n_max=None):
    """ψ(ad_z)·ξ, ψ(s)=s/(e^s−1)=Σ B_n s^n/n!；adjoint=True 时用转置算子
    n_max: 级数项数上限（仿真快速档用 14，全精度用全部 30 项）"""
    L = ad_map_T(z6) if adjoint else ad_map(z6)
    out = np.zeros(6); term = np.asarray(xi6, dtype=float).copy()
    inv_fact = 1.0
    for n in range(len(BERN) if n_max is None else n_max):
        if n >= 2:
            inv_fact /= n
        out += BERN[n]*inv_fact*term
        term = L(term)
    return out

def psi_ad(z6, xi6):
    return psi_apply(z6, xi6, adjoint=False)

def shaped_feedback_fast(z6, w6):
    return psi_apply(z6, w6, adjoint=True, n_max=14)

def A_log_mat(z6):
    M = np.zeros((6, 6))
    for i in range(6):
        e = np.zeros(6); e[i] = 1.0
        M[:, i] = psi_ad(z6, e)
    return M

def shaped_feedback(z6, w6):
    """A_log^T·w 无需显式矩阵: ψ(ad_z)^T w = ψ 经转置算子作用的级数"""
    return psi_apply(z6, w6, adjoint=True)

print("="*78)
print("A. DQ 对数 exp/log 往返一致性")
print("="*78)
max_err = 0.0
for _ in range(200):
    axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
    theta = rng.uniform(0.05, np.pi - 0.05)
    t = rng.uniform(-1, 1, 3)
    r = qexp_pure(pure(0.5*theta*axis))
    x = dq_from_rt(r, t)
    x2 = dq_exp_z(dq_log(x))
    err = max(np.linalg.norm(x2[0]-x[0]), np.linalg.norm(x2[1]-x[1]))
    max_err = max(max_err, err)
print(f"   200 组随机位姿 exp(log(x)) 往返最大误差 = {max_err:.2e}")

print()
print("="*78)
print("B. dexp^-1 恒等式: d/dt(2 ln x̃) = ψ(ad_z) ξ̃ （与中心差分对比）")
print("="*78)
# 轨迹: x̃(t) = Exp(ξ̃ t)·x̃0（Exp 的 2ln = ξ̃t，即 exp_G(½ξ̃t)）⇒ 2ẋ̃x̃* = ξ̃ 常值，精确解
for (theta0, scale) in [(2.5, 1.0), (0.8, 0.5), (3.1, 1.2)]:
    axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
    x0 = dq_from_rt(qexp_pure(pure(0.5*theta0*axis)), rng.uniform(-1, 1, 3)*scale)
    xi6 = rng.normal(size=6)
    t0, h = 0.37, 1e-6
    zp = dq_log(dq_mul(dq_exp_xi(xi6*(t0+h)), x0))
    zm = dq_log(dq_mul(dq_exp_xi(xi6*(t0-h)), x0))
    zdot_fd = (zp - zm) / (2*h)
    z_t = dq_log(dq_mul(dq_exp_xi(xi6*t0), x0))
    zdot_series = psi_ad(z_t, xi6)
    rel = np.linalg.norm(zdot_series - zdot_fd) / np.linalg.norm(zdot_fd)
    print(f"   θ0={theta0:.2f}: ‖ψ(ad_z)ξ̃ − 中心差分‖/‖差分‖ = {rel:.2e}")

print()
print("="*78)
print("C. A_log 结构与近恒等极限")
print("="*78)
axis = np.array([0.3, -0.5, 0.81]); axis /= np.linalg.norm(axis)
x = dq_from_rt(qexp_pure(pure(0.5*2.2*axis)), np.array([0.4, -0.2, 0.15]))
z = dq_log(x)
Alog = A_log_mat(z)
print(f"   A_log 左上块 vs Jl^-1(φ) 闭式最大差   = {np.max(np.abs(Alog[:3,:3] - Vinv_mat(z[:3]))):.2e}")
print(f"   A_log 右上块（理论为 0）最大绝对值    = {np.max(np.abs(Alog[:3,3:])):.2e}")
print(f"   A_log 右下块 vs Jl^-1(φ) 最大差       = {np.max(np.abs(Alog[3:,3:] - Vinv_mat(z[:3]))):.2e}")
x_small = dq_from_rt(qexp_pure(pure(0.5*1e-4*np.array([1.,0,0]))), np.array([1e-4,0,0]))
print(f"   近恒等: max|A_log − I|                = {np.max(np.abs(A_log_mat(dq_log(x_small)) - np.eye(6))):.2e}")
A0 = np.zeros((6,6)); A0[:3,:3] = -0.5*np.eye(3); A0[3:,3:] = np.eye(3)
print(f"   近恒等: max|A_论文 − diag(−I/2, I)|   = {np.max(np.abs(paper_A(x_small) - A0)):.2e}")

print()
print("="*78)
print("D. 旋转反馈幅值（纯旋转误差切片, K_{p,O}: C1 用 4k, 对数用 k）")
print("="*78)
k = 1.0
ax = np.array([0.2, 0.3, 0.933]); ax /= np.linalg.norm(ax)
print("   θ       |u_C1|/k    |u_log|/k    比值     （理论: sinθ vs θ）")
for deg in [30, 90, 120, 170, 179]:
    th = np.deg2rad(deg)
    x = dq_from_rt(qexp_pure(pure(0.5*th*ax)), np.zeros(3))
    A = paper_A(x)
    u_C1 = -A[:3,:3].T @ (4*k*(-vec3(x[0])))
    u_log = -shaped_feedback(dq_log(x), k*dq_log(x))
    print(f"   {deg:>4.0f}°   {np.linalg.norm(u_C1):9.4f}  {np.linalg.norm(u_log):9.4f}  "
          f"{np.linalg.norm(u_log)/max(np.linalg.norm(u_C1),1e-12):8.2f}")

print("\n   6D（有限旋转+平移）: 精确整形 u=-A_log^T k z 与裸对数 u=-kz（Chandra 式(35) 型）之差:")
for (th, tn) in [(1.5, 0.3), (2.5, 0.5), (0.2, 0.05)]:
    x = dq_from_rt(qexp_pure(pure(0.5*th*ax)), np.array([0.3, -0.2, 0.1])*(tn/0.3))
    z = dq_log(x)
    diff = -shaped_feedback(z, k*z) - (-k*z)
    print(f"   θ={th:.2f}, ‖t‖={tn:.2f}: ‖Δu‖={np.linalg.norm(diff):.4f} "
          f"(旋转通道 {np.linalg.norm(diff[:3]):.4f}, 平移通道 {np.linalg.norm(diff[3:]):.4f})")

# ----------------------------------------------------------------------
# E. 闭环误差动力学仿真（d=0，反馈线性化后的 6D 误差模型）
#    等局部刚度 DC=80: C1 取 K_{p,O}=4·80, K_{p,T}=80；LOG 取 K_p=80·I
#    等阻尼极点 {-4,-20} ⇒ K_d = 24 I
# ----------------------------------------------------------------------
def simulate(ctrl, theta0=3.0, T=4.0, dt=5e-4, lam_d=0.0):
    t_axis = np.array([0.0, 1.0, 0.0])
    r0 = qexp_pure(pure(0.5*theta0*t_axis))
    x = dq_from_rt(r0, np.array([0.15, -0.1, 0.12]))
    e_xi = np.zeros(6)
    kd, kstiff = 24.0, 80.0
    Kd = kd*np.eye(6)
    Kp = np.diag([4*kstiff]*3 + [kstiff]*3) if ctrl == 'C1' else kstiff*np.eye(6)
    N = int(T/dt)
    th_hist = np.zeros(N); V_hist = np.zeros(N); u_hist = np.zeros(N)
    w_hist = np.zeros(N); tr_hist = np.zeros(N); Vdot_err = 0.0
    def closed_loop(xv, ex):
        if ctrl == 'C1':
            u = -Kd@ex - paper_A(xv).T@Kp@ez_paper(xv)
        else:
            Kde = Kd if ctrl == 'log' else Kd + lam_d*(1 - xv[0][0])*np.eye(6)
            u = -Kde@ex - shaped_feedback_fast(dq_log(xv), Kp@dq_log(xv))
        return u
    def state_dot(state):
        r = state[:4]; d = state[4:8]; ex = state[8:]
        xv = (r, d)
        uu = closed_loop(xv, ex)
        xi_p = (pure(0.5*ex[:3]), pure(0.5*ex[3:]))
        dx = dq_mul(xi_p, xv)
        return np.concatenate([dx[0], dx[1], uu])
    def V_of(state, ctrl=ctrl):
        xv = (state[:4], state[4:8]); ex = state[8:]
        ez = ez_paper(xv) if ctrl == 'C1' else dq_log(xv)
        return 0.5*ex@ex + 0.5*ez@Kp@ez
    hstep = 1e-6
    for i in range(N):
        ez = ez_paper(x) if ctrl == 'C1' else dq_log(x)
        u = closed_loop(x, e_xi)
        th_hist[i] = 2*np.arccos(np.clip(x[0][0], -1, 1))
        V_hist[i] = 0.5*e_xi@e_xi + 0.5*ez@Kp@ez
        u_hist[i] = np.linalg.norm(u)
        w_hist[i] = np.linalg.norm(e_xi)
        tr_hist[i] = np.linalg.norm(dq_translation(x))
        state = np.concatenate([x[0], x[1], e_xi])
        if i % 1000 == 0 and i > 0:      # 逐点校验 V̇ = −e_ξ^T K_d e_ξ（前向单步差分）
            Vp = V_of(state + dt*state_dot(state))
            Vdot_fd = (Vp - V_hist[i])/dt
            Kde = Kd if ctrl == 'log' else (Kd if ctrl == 'C1' else Kd + lam_d*(1 - x[0][0])*np.eye(6))
            Vdot_th = -e_xi@(Kde@e_xi)
            Vdot_err = max(Vdot_err, abs(Vdot_fd - Vdot_th)/max(abs(Vdot_th), 1.0))
        k1 = state_dot(state); k2 = state_dot(state + 0.5*dt*k1)
        k3 = state_dot(state + 0.5*dt*k2); k4 = state_dot(state + dt*k3)
        state = state + dt/6*(k1 + 2*k2 + 2*k3 + k4)
        r_n = state[:4]/np.linalg.norm(state[:4])
        x = (r_n, state[4:8]); e_xi = state[8:]
    return np.arange(N)*dt, th_hist, V_hist, u_hist, w_hist, tr_hist, Vdot_err

print()
print("="*78)
print("E. 等局部刚度（DC=80）、等阻尼（极点{-4,-20}）大误差收敛仿真, θ0=3.0 rad")
print("="*78)
results = {}
for name, ctrl, lam in [('C1  (论文式5.2反馈)', 'C1', 0.0),
                        ('LOG (对数+A_log整形)', 'log', 0.0),
                        ('LOG+VD (误差相关阻尼λd=60)', 'log_vd', 60.0)]:
    t, th, V, u, w, tr, verr = simulate(ctrl, theta0=3.0, T=4.0, lam_d=lam)
    results[name] = (t, th, V, u, w, tr)
    idx_arr = np.where(th < 0.05)[0]
    tc = t[idx_arr[0]] if len(idx_arr) else float('inf')
    dV = np.diff(V)
    viol = dV.max() if dV.size and dV.max() > 1e-9 else 0.0
    # 平移过冲：峰值平移误差 / 初始平移误差
    ovs = tr.max()/np.linalg.norm([0.15, -0.1, 0.12])
    print(f"   {name:30s} θ→0.05rad 时间 = {tc:6.3f} s | max ΔV(应≈0) = {viol:.1e} | "
          f"max‖u‖ = {u.max():7.1f} | max‖e_ξ‖ = {w.max():6.2f} | 平移过冲比 = {ovs:.2f} | "
          f"V̇校验最大相对偏差 = {verr:.1e}")
t1, th1, _, _, _, _ = results['C1  (论文式5.2反馈)']
t2, th2, _, _, _, _ = results['LOG (对数+A_log整形)']
t3, th3, _, _, _, _ = results['LOG+VD (误差相关阻尼λd=60)']
print("\n   θ(t) 对比（度）:")
print("   t(s)     C1        LOG       LOG+VD")
for tt in [0.1, 0.25, 0.5, 1.0, 2.0]:
    i1 = min(np.searchsorted(t1, tt), len(t1)-1)
    i2 = min(np.searchsorted(t2, tt), len(t2)-1)
    i3 = min(np.searchsorted(t3, tt), len(t3)-1)
    print(f"   {tt:4.2f}  {np.rad2deg(th1[i1]):8.2f}  {np.rad2deg(th2[i2]):8.2f}  {np.rad2deg(th3[i3]):8.2f}")

# 近恒等一致性：小误差下 C1 与 LOG 响应应几乎重合
print("\n   近恒等一致性检查 (θ0 = 0.20 rad):")
for name, ctrl, lam in [('C1', 'C1', 0.0), ('LOG', 'log', 0.0)]:
    t, th, V, u, w, tr, verr = simulate(ctrl, theta0=0.20, T=1.5, lam_d=lam)
    idx_arr = np.where(th < 0.01)[0]
    tc = t[idx_arr[0]] if len(idx_arr) else float('inf')
    print(f"     {name}: θ→0.01rad 时间 = {tc:.3f} s, max‖u‖ = {u.max():.1f}, max‖e_ξ‖ = {w.max():.2f}")

# ----------------------------------------------------------------------
# F. 力矩最优映射 KKT 验证
# ----------------------------------------------------------------------
print()
print("="*78)
print("F. 力矩最优映射（min ‖M q̈+h‖² s.t. J q̈ = b, 6×8 冗余）KKT 验证")
print("="*78)
for trial in range(3):
    n = 8
    Rm = rng.normal(size=(n, n)); M = Rm@Rm.T + 6*np.eye(n)   # 关节空间惯量 n×n
    J = rng.normal(size=(6, n))
    h = rng.normal(size=n)
    a_cmd = rng.normal(size=6); Jdot_qdot = rng.normal(size=6)
    b = a_cmd - Jdot_qdot
    Minv = np.linalg.inv(M); Minv2 = Minv@Minv
    qdd_opt = Minv2@J.T@np.linalg.solve(J@Minv2@J.T, b + J@Minv@h) - Minv@h
    tau_opt = M@qdd_opt + h
    qdd_jp = np.linalg.pinv(J)@b
    tau_jp = M@qdd_jp + h
    res = np.linalg.norm(J@qdd_opt - b)
    # KKT 驻点: 2Mᵀ(Mq̈+h) + Jᵀλ = 0 ⟺ Mτ ∈ Range(Jᵀ)（τ=Mq̈+h, M 对称）
    lam = np.linalg.lstsq(J.T, M@tau_opt, rcond=None)[0]
    kkt_res = np.linalg.norm(M@tau_opt - J.T@lam)
    print(f"   trial {trial}: 约束残差 = {res:.1e} | KKT 残差 = {kkt_res:.1e} | "
          f"‖τ‖opt = {np.linalg.norm(tau_opt):7.3f}  ‖τ‖J+ = {np.linalg.norm(tau_jp):7.3f}  "
          f"降幅 = {100*(1-np.linalg.norm(tau_opt)/np.linalg.norm(tau_jp)):5.1f}%")

print()
print("全部验证完成。")
