#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
误差坐标族底层结构验证：弦（论文 C1）/ 弧（对数）/ 割圆（对偶 Cayley）
====================================================================
目的：把"误差体系 = 图册 + 势函数 + 梯度输运"这一层结构用数值钉死。

验证项：
  G1. 对偶 Cayley 图册的代数定义与恒等式：
      ζ = 2(x̃−1)(x̃+1)^{-1}（DQ 除法，纯代数）
      旋转部 = 2σ（σ = μ/(1+η) = tan(θ/4)·l，标准 MRP 恒等式）
  G2. 对偶 Cayley 图册的运动学 ζ̇ = A_cay(x̃)·ξ̃（空间 twist 约定，DQ 乘法闭式）
      与中心差分对比；A_cay 沿轴增益 = ½·sec²(θ/4)（随 θ 增长，2π 处奇异）
  G3. 三族势函数与沿轴反馈（等局部刚度 k 配平）：
      弦   U = 2k·sin²(θ/2)          （K_{p,O}=4k）  g = k·sinθ        图册: 全局，势函数有界
      弧   U = ½k·θ²                 （K_p = k）     g = k·θ           图册: θ<π
      割圆 U = 8k·tan²(θ/4)          （K = 4k）      g = 4k·tan(θ/4)sec²(θ/4)  图册: θ<2π
      用 dU/dθ 的解析式与有限差分互检；验证三族 U''(0)=k（等局部刚度）
  G4. 测地线参考在对数图册中是直线：x_d(t)=Exp(ξ_d t) ⟹ z_d(t)=ξ_d·t
仅依赖 numpy。
"""
import numpy as np

rng = np.random.default_rng(20260925)
np.set_printoptions(precision=4, suppress=True)

def pure(v):
    return np.concatenate([[0.0], np.asarray(v, dtype=float)])

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

def qexp_pure(w):
    th = np.linalg.norm(vec3(w))
    if th < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0]) + 0.5*w
    v = vec3(w)
    return np.concatenate([[np.cos(th)], np.sin(th)/th * v])

def dq_mul(x, y):
    r1, d1 = x; r2, d2 = y
    return qmul(r1, r2), (qmul(r1, d2) + qmul(d1, r2))

def dq_conj(x):
    r, d = x
    return qconj(r), qconj(d)

ONE = np.array([1.0, 0.0, 0.0, 0.0])   # 四元数乘法单位元（勿用标量 1 逐分量加减！）

def dq_inv(x):
    """一般 DQ 逆：a = a0 + εa1, a^{-1} = (a0* − ε a0* a1 a0*)/|a0|²"""
    r, d = x
    n2 = r@r
    r_inv = qconj(r)/n2
    d_inv = -qmul(qmul(r_inv, d), r_inv)
    return r_inv, d_inv

def dq_from_rt(r, t3):
    return r, 0.5*qmul(pure(t3), r)

def dq_translation(x):
    r, d = x
    return vec3(2*qmul(d, qconj(r)))

def bracket(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def V_mat(phi):
    th = np.linalg.norm(phi)
    P = bracket(phi)
    if th < 1e-8:
        return np.eye(3) + 0.5*P + (1.0/6)*P@P
    return np.eye(3) + (1-np.cos(th))/th**2*P + (th-np.sin(th))/th**3*(P@P)

def Vinv_mat(phi):
    th = np.linalg.norm(phi)
    P = bracket(phi)
    if th < 1e-8:
        return np.eye(3) - 0.5*P + (1.0/12)*P@P
    c = (1 - (th/2)/np.tan(th/2))/th**2
    return np.eye(3) - 0.5*P + c*(P@P)

def dq_log(x):
    r, d = x
    eta = r[0]; mu = vec3(r)
    s = np.linalg.norm(mu)
    theta = 2*np.arctan2(s, eta)
    phi = theta*mu/s if s > 1e-12 else np.zeros(3)
    return np.concatenate([phi, Vinv_mat(phi)@dq_translation(x)])

def dq_exp_z(z6):
    phi = z6[:3]; rho = z6[3:]
    return dq_from_rt(qexp_pure(pure(0.5*phi)), V_mat(phi)@rho)

# ----------------------------------------------------------------------
print("="*78)
print("G1. 对偶 Cayley 图册 ζ = 2(x̃−1)(x̃+1)^{-1}：代数恒等式")
print("="*78)
# 旋转部应为 2σ，σ = μ/(1+η) = tan(θ/4)·l（非 shadow 分支 MRP）
max_err_id = 0.0
max_err_tan = 0.0
for _ in range(200):
    axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
    theta = rng.uniform(0.05, np.pi - 0.02)         # 避开 2π 奇异点即可，这里测到 π
    t = rng.uniform(-1, 1, 3)
    r = qexp_pure(pure(0.5*theta*axis))
    x = dq_from_rt(r, t)
    xm1 = (x[0]-ONE, x[1])
    xp1_inv = dq_inv((x[0]+ONE, x[1]))
    zeta = dq_mul(xm1, xp1_inv)
    zeta = (2*zeta[0], 2*zeta[1])
    sigma = vec3(r)/(1 + r[0])
    err_id = np.linalg.norm(vec3(zeta[0]) - 2*sigma)
    max_err_id = max(max_err_id, err_id)
    # 恒等式 |ζ_real| = 2 tan(θ/4)
    err_tan = abs(np.linalg.norm(vec3(zeta[0])) - 2*np.tan(theta/4))
    max_err_tan = max(max_err_tan, err_tan)
print(f"   200 组随机位姿：|ζ旋转部 − 2σ| 最大误差          = {max_err_id:.2e}")
print(f"   200 组随机位姿：‖ζ旋转部‖ − 2tan(θ/4) 最大误差   = {max_err_tan:.2e}")
print("   ⟹ ζ = 2(x̃−1)(x̃+1)^{-1} 的旋转部恰为 2×MRP（tan(θ/4) 族），全程 DQ 除法，纯代数 ✓")

# ----------------------------------------------------------------------
print()
print("="*78)
print("G2. 对偶 Cayley 图册运动学 ζ̇ = A_cay(x̃)·ξ̃（DQ 闭式 vs 中心差分）")
print("="*78)
def zeta_of(x):
    xm1 = (x[0]-ONE, x[1])
    z = dq_mul(xm1, dq_inv((x[0]+ONE, x[1])))
    return np.concatenate([vec3(2*z[0]), vec3(2*z[1])])

def zeta_kin_closed(x, xi6, h=None):
    """ζ̇ 的 DQ 闭式: ζ̇ = 2[ ẋ̃(x̃+1)^{-1} − (x̃−1)(x̃+1)^{-1} ẋ̃ (x̃+1)^{-1} ],
       ẋ̃ = ½ ξ̃ x̃（空间 twist 约定）"""
    r, d = x
    xi = (pure(0.5*xi6[:3]), pure(0.5*xi6[3:]))
    xdot = dq_mul(xi, x)                     # ½ ξ̃ x̃（ξ̃ 纯 DQ 与 ½ 因子合并）
    xp1_inv = dq_inv((r+ONE, d))
    term1 = dq_mul(xdot, xp1_inv)
    xm1 = (r-ONE, d)
    term2 = dq_mul(dq_mul(xm1, xp1_inv), dq_mul(xdot, xp1_inv))
    out = dq_mul((term1[0]-term2[0], term1[1]-term2[1]), (np.array([2.,0,0,0]), np.zeros(4)))
    return np.concatenate([vec3(out[0]), vec3(out[1])])

for (theta0, tn) in [(1.2, 0.3), (2.8, 0.4), (0.4, 0.1)]:
    axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
    x0 = dq_from_rt(qexp_pure(pure(0.5*theta0*axis)), rng.uniform(-1, 1, 3)*tn/0.3)
    xi6 = rng.normal(size=6)
    # FD: ζ(x̃(t)) 沿 ξ̃ 常值轨迹, x̃(t) = Exp(ξ̃t)·x̃0
    h = 1e-6
    def x_at(tt):
        return dq_mul(dq_exp_z(xi6*tt), x0)
    zp = zeta_of(x_at(h)); zm = zeta_of(x_at(-h))
    zdot_fd = (zp - zm)/(2*h)
    zdot_cl = zeta_kin_closed(x0, xi6)
    rel = np.linalg.norm(zdot_cl - zdot_fd)/np.linalg.norm(zdot_fd)
    print(f"   θ0={theta0:.2f}: ‖A_cay ξ̃ − 中心差分‖/‖差分‖ = {rel:.2e}")

# 沿轴增益：θ 方向自旋 ⟹ ζ̇ 轴向分量 = ½·sec²(θ/4)·θ̇
print("\n   A_cay 沿轴增益（理论 ½·sec²(θ/4)，随 θ 增长，θ→2π 奇异）:")
ax = np.array([0.2, 0.3, 0.933]); ax /= np.linalg.norm(ax)
for deg in [30, 90, 150, 210, 300]:
    th = np.deg2rad(deg)
    x = dq_from_rt(qexp_pure(pure(0.5*th*ax)), np.zeros(3))
    g_num = np.linalg.norm(zeta_kin_closed(x, np.concatenate([ax, np.zeros(3)])))   # 单位自旋
    g_th = 0.5/np.cos(th/4)**2
    print(f"   θ={deg:>3.0f}°  数值 {g_num:7.4f}  理论 {g_th:7.4f}")

# ----------------------------------------------------------------------
print()
print("="*78)
print("G3. 三族势函数与沿轴反馈（等局部刚度 k），dU/dθ 解析 vs 差分")
print("="*78)
k = 1.0
def U_chord(th): return 2*k*np.sin(th/2)**2          # K_{p,O} = 4k
def g_chord(th): return k*np.sin(th)
def U_arc(th):   return 0.5*k*th**2
def g_arc(th):   return k*th
def U_cay(th):   return 8*k*np.tan(th/4)**2          # K = 4k（ζ ≈ ½θ 局部因子）
def g_cay(th):   return 4*k*np.tan(th/4)/np.cos(th/4)**2

h = 1e-6
print("   U''(0) 等局部刚度检查（应均为 k）:")
for name, U in [('弦(4k)', U_chord), ('弧', U_arc), ('割圆(4k)', U_cay)]:
    U2 = (U(h) - 2*U(0) + U(-h))/h**2
    print(f"     {name:8s} U''(0) = {U2:9.6f}")
print("\n   θ     弦势U   弧势U   割圆势U | 弦反馈g  弧反馈g  割圆反馈g   （×k）")
for deg in [30, 90, 150, 170, 250]:
    th = np.deg2rad(deg)
    # 差分验证解析 dU/dθ
    for U, g in [(U_chord, g_chord), (U_arc, g_arc), (U_cay, g_cay)]:
        fd = (U(th+h) - U(th-h))/(2*h)
        assert abs(fd - g(th)) < 1e-9*max(1, abs(g(th))) or abs(fd - g(th)) < 1e-7, (deg, fd, g(th))
    print(f"   {deg:>3.0f}°  {U_chord(th):6.3f}  {U_arc(th):6.3f}  {U_cay(th):7.3f}  | "
          f"{g_chord(th):7.4f} {g_arc(th):7.4f} {g_cay(th):9.4f}")
print("   （解析 dU/dθ 与中心差分全部一致；弦族势函数上限 2k，弧族在 π 处 4.93k，割圆族在 π 处 8k 且 θ→2π 发散）")
print("   图册域: 弦=全局(光滑但饱和) | 弧=θ<π(割迹跳变) | 割圆=θ<2π(对径点奇异, 纯代数)")

# ----------------------------------------------------------------------
print()
print("="*78)
print("G4. 测地线参考在对数图册中是直线（TODQ 常值 twist 参考生成 = Exp(ξ_d t)）")
print("="*78)
xi_d = rng.normal(size=6)
max_dev = 0.0
for tt in np.linspace(0.01, 2.0, 20):
    zd = dq_log(dq_exp_z(xi_d*tt))
    max_dev = max(max_dev, np.linalg.norm(zd - xi_d*tt))
print(f"   20 个采样点 ‖2ln Exp(ξ_d t) − ξ_d t‖ 最大偏差 = {max_dev:.2e}  ✓（图册内仿射）")
print("\n全部结构验证完成。")
