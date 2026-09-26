#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
速度级固定增益对数控制器：两种设计的结构对比验证
==================================================
设计 1（裸反馈, Chandra/Wang-Yu 型）:  ξ̃_cmd = -K z
设计 2（结构化, 本文提出）:          ξ̃_cmd = dexp_z(-K z)   （图表内规定线性衰减 + 逆输运）

运动学: x̃̇ = ½ ξ̃ x̃ （twist 级, J=I 抽象）,  z = 2 ln x̃,  ż = ψ(ad_z) ξ̃

验证项:
  A. 旋转切片: 裸反馈 u=-kφ 下 θ(t)=θ₀e^{-kt} 精确成立（沿轴不变性 ⟹ 旋转通道满速率）
  B. 6D: 结构化设计下 z(t)=e^{-Kt}z₀ 精确成立（全图册一致指数, 旋转+平移两通道）
  C. 6D: 裸设计下平移通道有效衰减率退化（横向因子 sinθ/θ）+ 旋转误差经 W 块泵入平移通道
  D. 结构化设计的 twist 需求有界性: 旋转块 ‖Jl(φ)‖ ≤ 1（非扩张）
"""
import numpy as np
from math import factorial

rng = np.random.default_rng(7)
np.set_printoptions(precision=4, suppress=True)

def pure(v): return np.concatenate([[0.0], np.asarray(v, float)])
def vec3(q): return q[1:]
def qmul(a, b):
    w1,x1,y1,z1 = a; w2,x2,y2,z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])
def qconj(a): return a*np.array([1.,-1,-1,-1])
def qexp_pure(w):
    th = np.linalg.norm(vec3(w))
    if th < 1e-12: return np.array([1.,0,0,0]) + 0.5*w
    v = vec3(w); return np.concatenate([[np.cos(th)], np.sin(th)/th*v])
def dq_mul(x, y):
    r1,d1 = x; r2,d2 = y
    return qmul(r1,r2), (qmul(r1,d2)+qmul(d1,r2))
def dq_translation(x):
    r,d = x; return vec3(2*qmul(d, qconj(r)))
def dq_from_rt(r, t3): return r, 0.5*qmul(pure(t3), r)
def bracket(v): return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
def V_mat(phi):
    th = np.linalg.norm(phi); P = bracket(phi)
    if th < 1e-8: return np.eye(3) + 0.5*P + (1/6)*P@P
    return np.eye(3) + (1-np.cos(th))/th**2*P + (th-np.sin(th))/th**3*(P@P)
def Vinv_mat(phi):
    th = np.linalg.norm(phi); P = bracket(phi)
    if th < 1e-8: return np.eye(3) - 0.5*P + (1/12)*P@P
    return np.eye(3) - 0.5*P + ((1-(th/2)/np.tan(th/2))/th**2)*(P@P)
def dq_log(x):
    r,d = x; eta=r[0]; mu=vec3(r); s=np.linalg.norm(mu)
    theta = 2*np.arctan2(s, eta)
    phi = theta*mu/s if s > 1e-12 else np.zeros(3)
    return np.concatenate([phi, Vinv_mat(phi)@dq_translation(x)])
def dq_exp_z(z6):
    phi, rho = z6[:3], z6[3:]
    return dq_from_rt(qexp_pure(pure(0.5*phi)), V_mat(phi)@rho)

def ad_map(z6):
    phi, rho = z6[:3], z6[3:]
    def L(xi6):
        w,v = xi6[:3], xi6[3:]
        return np.concatenate([np.cross(phi,w), np.cross(phi,v)-np.cross(w,rho)])
    return L

def bernoulli_numbers(N):
    B = np.zeros(N+1); B[0] = 1.0
    from math import comb
    for m in range(1, N+1):
        B[m] = -sum(comb(m+1, k)*B[k] for k in range(m))/(m+1)
    return B
BERN = bernoulli_numbers(30)           # 递推生成（勿手抄!），B1=−1/2 约定

def psi_apply(z6, xi6, n=30):          # ψ(ad_z)ξ, ψ(s)=s/(e^s−1)
    L = ad_map(z6); out = np.zeros(6); term = np.asarray(xi6, float).copy()
    for k in range(n):
        out += BERN[k]/float(factorial(k))*term
        term = L(term)
    return out

def dexp_apply(z6, w6, n=30):          # dexp_z(w) = Σ ad_z^k w/(k+1)!  （ψ 的逆）
    L = ad_map(z6); out = np.zeros(6); term = np.asarray(w6, float).copy()
    for k in range(n):
        out += term/float(factorial(k+1))
        term = L(term)
    return out

K = 2.0*np.eye(6)                       # 固定增益（图表内各向同性）
k = 2.0

print("="*76)
print("A. 旋转切片: 裸反馈 u=-kφ 下 θ(t)=θ₀e^{-kt} 是否精确成立")
print("="*76)
axis = np.array([0.2,0.3,0.933]); axis/=np.linalg.norm(axis)
theta0 = 2.5
x = dq_from_rt(qexp_pure(pure(0.5*theta0*axis)), np.zeros(3))
dt = 1e-3; T = 2.0
max_dev = 0.0
for i in range(int(T/dt)):
    z = dq_log(x)
    u = -k*z[:3]                        # 裸反馈（纯旋转切片下 z=[φ;0]）
    def f(st):
        r = st[:4]
        dx = 0.5*qmul(pure(u[:3]), r)
        return np.concatenate([dx, np.zeros(4)])
    st = np.concatenate([x[0],x[1]])
    k1=f(st);k2=f(st+dt/2*k1);k3=f(st+dt/2*k2);k4=f(st+dt*k3)
    st = st + dt/6*(k1+2*k2+2*k3+k4)
    r=st[:4]/np.linalg.norm(st[:4]); x=(r, st[4:8])
    th = 2*np.arctan2(np.linalg.norm(vec3(x[0])), x[0][0])
    th_exact = theta0*np.exp(-k*(i+1)*dt)
    max_dev = max(max_dev, abs(th-th_exact))
print(f"   2s 内 |θ(t) − θ₀e^{{-kt}}| 最大偏差 = {max_dev:.2e}  ✓ 旋转通道裸反馈即精确指数（满速率 k）")

print()
print("="*76)
print("B/C. 6D 对比: 裸设计 vs 结构化设计（θ₀=2.5, ‖ρ₀‖≈0.5, k=2）")
print("="*76)
def simulate(design, T=2.5, dt=2e-4):
    x = dq_from_rt(qexp_pure(pure(0.5*2.5*axis)), np.array([0.3,-0.2,0.25]))
    N=int(T/dt)
    th=np.zeros(N); rho=np.zeros(N); zn=np.zeros(N); uN=np.zeros(N)
    resid_max = 0.0
    for i in range(N):
        z = dq_log(x)
        th[i]=np.linalg.norm(z[:3]); rho[i]=np.linalg.norm(z[3:]); zn[i]=np.linalg.norm(z)
        if design=='naive':
            xi = -K@z
        else:
            xi = dexp_apply(z, -K@z)
            # 残差校验: ψ(ad_z)·ξ_cmd 应精确等于 -Kz
            resid_max = max(resid_max, np.linalg.norm(psi_apply(z, xi) + K@z))
        uN[i]=np.linalg.norm(xi)
        def f(st):
            xv=(st[:4],st[4:8]); xi6=xi
            dx = dq_mul((pure(0.5*xi6[:3]), pure(0.5*xi6[3:])), xv)
            return np.concatenate([dx[0],dx[1]])
        st=np.concatenate([x[0],x[1]])
        k1=f(st);k2=f(st+dt/2*k1);k3=f(st+dt/2*k2);k4=f(st+dt*k3)
        st=st+dt/6*(k1+2*k2+2*k3+k4)
        r=st[:4]/np.linalg.norm(st[:4]); x=(r,st[4:8])
    return np.arange(N)*dt, th, rho, zn, uN, resid_max

t_n, th_n, rho_n, zn_n, u_n, _ = simulate('naive')
t_s, th_s, rho_s, zn_s, u_s, resid = simulate('structured')
print(f"   结构化设计残差 max‖ψ(ad_z)ξ_cmd + Kz‖ = {resid:.1e}  （应≈0: 图表内动力学=精确线性）")

# 裸设计平移通道有效衰减率（对 ‖ρ‖ 半对数拟合后半段斜率）
mask = (t_n>0.8)&(t_n<2.0)
rate_n = -np.polyfit(t_n[mask], np.log(rho_n[mask]+1e-300), 1)[0]
mask2 = (t_s>0.4)&(t_s<1.5)
rate_s = -np.polyfit(t_s[mask2], np.log(rho_s[mask2]+1e-300), 1)[0]
print(f"   平移通道有效衰减率: 裸设计 {rate_n:.3f} vs 结构化 {rate_s:.3f} (名义 k=2)")
print(f"   理论横向因子 sin(θ)/θ @θ=2.5: {np.sin(2.5)/2.5:.3f}  → 2×该值≈{2*np.sin(2.5)/2.5:.3f}（量级吻合）")
print("\n   t(s)   ‖z‖裸   ‖z‖结构化   θ裸(°)  θ结构化(°)   ‖ρ‖裸   ‖ρ‖结构化   ‖ξ_cmd‖裸  ‖ξ_cmd‖结构化")
for tt in [0.1,0.3,0.6,1.0,1.6,2.4]:
    i=int(tt/2.5e-4)-1
    print(f"   {tt:4.1f}  {zn_n[i]:6.3f}  {zn_s[i]:8.3f}   {np.rad2deg(th_n[i]):6.2f}  {np.rad2deg(th_s[i]):8.2f}    "
          f"{rho_n[i]:6.3f}  {rho_s[i]:8.3f}     {u_n[i]:6.3f}   {u_s[i]:6.3f}")

print()
print("="*76)
print("D. 结构化设计 twist 需求的旋转块非扩张性: ‖Jl(φ)‖ ≤ 1")
print("="*76)
mx = 0.0
for _ in range(500):
    ax = rng.normal(size=3); ax/=np.linalg.norm(ax)
    th = rng.uniform(0.01, np.pi-0.01)
    Jl = V_mat(th*ax)
    mx = max(mx, np.linalg.norm(Jl, 2))
print(f"   500 组随机 (θ∈(0,π)): max‖Jl(φ)‖₂ = {mx:.4f}  ≤ 1 ✓ （特征值 {1:.0f} 与 2sin(θ/2)/θ∈[2/π,1]）")
print(f"   对照: 弦族传输逆 ‖A₁₁⁻¹‖ = 2/η̃ 在 θ→π 发散（附录 C.2），对数逆 dexp 旋转块恒 ≤ 1")
