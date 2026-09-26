"""Reproduce the mathematical checks in the accompanying theory design note.

Offline error-model experiments only: no robot, ROS, or hardware imports.
Run: python3 docs/theory_validation_20260919/verify_design.py
"""
from pathlib import Path
import json
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import linprog
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
RNG = np.random.default_rng(20260919)


def skew(x):
    x, y, z = x
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def blocks(q, T):
    x = np.r_[2*q[1:], T]
    B = np.block([[q[0]*np.eye(3)-skew(x[:3])/2, np.zeros((3, 3))],
                  [-skew(T), np.eye(3)]])
    return x, B


def norms(z):
    return np.array([np.linalg.norm(z[:3]), np.linalg.norm(z[3:])])


def unit(z):
    return z / max(np.linalg.norm(z), 1e-14)


def make_state(x, s, a):
    q = np.r_[np.sqrt(1-np.dot(x[:3], x[:3])/4), x[:3]/2]
    return np.r_[q, x[3:], s-np.repeat(a, 3)*x]


def dynamics(a, b, disturbance, law="filtered"):
    aa, bb = np.repeat(a, 3), np.repeat(b, 3)
    def rhs(t, y):
        q, T, v = y[:4], y[4:7], y[7:]
        x, B = blocks(q, T)
        s = v+aa*x
        u = -aa*(B@v)-bb*s
        if law == "original":
            u = -(aa+bb)*v-aa*bb*x*np.r_[np.full(3,q[0]),np.ones(3)]
        d = disturbance(t, x, s, u)
        qdot = np.r_[-np.dot(q[1:], v[:3])/2,
                     (q[0]*v[:3]+np.cross(v[:3], q[1:]))/2]
        return np.r_[qdot, (B@v)[3:], u+d]
    return rhs


def integrate(a, b, disturbance, y0, duration=12., dt=.02, law="filtered"):
    ts = np.linspace(0, duration, round(duration/dt)+1)
    sol = solve_ivp(dynamics(a, b, disturbance, law), [0, duration], y0,
                    t_eval=ts, rtol=2e-9, atol=2e-11, max_step=.05)
    assert sol.success
    y = sol.y.T
    x = np.c_[2*y[:, 1:4], y[:, 4:7]]
    v = y[:, 7:]
    s = v+x*np.repeat(a, 3)
    nr = lambda z: np.c_[np.linalg.norm(z[:, :3], axis=1),
                         np.linalg.norm(z[:, 3:], axis=1)]
    return dict(t=ts, y=y, x=x, s=s, nx=nr(x), ns=nr(s),
                qnorm_error=float(np.max(abs(np.linalg.norm(y[:, :4], axis=1)-1))))


def identity_checks(n=2000):
    errors = np.zeros(4)
    for _ in range(n):
        q = RNG.normal(size=4); q /= np.linalg.norm(q)
        if q[0] < 0: q *= -1
        T, v, d = RNG.normal(size=3), RNG.normal(size=6), RNG.normal(size=6)
        a, b = RNG.uniform(.5, 12, 2), RNG.uniform(.5, 15, 2)
        aa, bb = np.repeat(a, 3), np.repeat(b, 3)
        x, B = blocks(q, T)
        O = -q[1:]
        A = np.block([[-(q[0]*np.eye(3)+skew(O))/2, np.zeros((3,3))],
                      [-skew(T), np.eye(3)]])
        H = np.diag([-2., -2., -2., 1., 1., 1.])
        s = v+aa*x; u = -aa*(B@v)-bb*s
        ds = u+d+aa*(B@v)
        # Independently use quaternion derivative and translation derivative.
        eta_dot = -np.dot(q[1:], v[:3])/2
        Udot = -4*eta_dot+T@((B@v)[3:])
        target = -np.dot(x, aa*x)+np.dot(x, s)
        errors = np.maximum(errors, [np.max(abs(B-H@A)),
                                     np.max(abs(ds-(-bb*s+d))),
                                     abs(Udot-target),
                                     abs(np.linalg.norm(B[:3,:3], 2)-1)])
    assert errors.max() < 1e-10
    return dict(samples=n, B_identity=float(errors[0]), filtered_identity=float(errors[1]),
                storage_identity=float(errors[2]), rotation_B_norm=float(errors[3]))


def frequency_checks():
    rows=[]
    for a,b in [(3.,3.),(3.,6.),(3.,12.)]:
        w=np.unique(np.r_[0., np.logspace(-6, 5, 50000), np.sqrt(a*b)])
        Gx=1/((1j*w+a)*(1j*w+b)); Gv=1j*w*Gx
        rows.append(dict(a=a,b=b,pose_numeric=float(abs(Gx).max()),pose_exact=1/(a*b),
                         twist_numeric=float(abs(Gv).max()),twist_exact=1/(a+b)))
    return rows


def additive_checks():
    a=np.array([3.,3.]); b=np.array([6.,6.]); D=np.array([.72,.18])
    E=D/(a*b); S=D/b
    rows=[]; plotted=None
    for j in range(12):
        if j == 0:
            x0=s0=np.zeros(6)
            disturbance=lambda t,x,s,u: np.array([0.,0.,D[0],D[1],0.,0.])
        else:
            x0=np.r_[unit(RNG.normal(size=3))*E[0],unit(RNG.normal(size=3))*E[1]]
            s0=np.r_[unit(RNG.normal(size=3))*S[0],unit(RNG.normal(size=3))*S[1]]
            f=RNG.uniform(.2, 8, 2); phase=RNG.uniform(0, 6, 2)
            def disturbance(t,x,s,u,f=f,phase=phase):
                return np.r_[D[0]*np.array([np.cos(f[0]*t+phase[0]),np.sin(f[0]*t+phase[0]),0]),
                             D[1]*np.array([np.sin(f[1]*t+phase[1]),0,np.cos(f[1]*t+phase[1])])]
        sim=integrate(a,b,disturbance,make_state(x0,s0,a))
        rows.append(dict(run=j,peak_pose_ratio=(sim['nx']/E).max(axis=0).tolist(),
                         peak_filtered_ratio=(sim['ns']/S).max(axis=0).tolist(),
                         quaternion_norm_error=sim['qnorm_error']))
        if j == 0: plotted=sim
    assert max(max(r['peak_pose_ratio']) for r in rows)<1+1e-6
    assert max(max(r['peak_filtered_ratio']) for r in rows)<1+1e-6
    fig, ax=plt.subplots(1,2,figsize=(10,3.5))
    for i, label in enumerate(['Rotation coordinate','Translation coordinate']):
        ax[i].plot(plotted['t'],plotted['nx'][:,i],label='Nonlinear response')
        ax[i].axhline(E[i],color='k',ls='--',label='D / (a b)')
        ax[i].set(xlabel='Time (s)',ylabel='Coordinate norm',title=label); ax[i].legend()
    fig.tight_layout(); fig.savefig(OUT/'additive_bound.png',dpi=170); plt.close(fig)
    return dict(a=a.tolist(),b=b.tolist(),D=D.tolist(),bound=E.tolist(),runs=rows,
                constant_final_ratio=(plotted['nx'][-1]/E).tolist(),
                constant_filtered_solution_error=float(np.max(abs(
                    plotted['s']-(1-np.exp(-6*plotted['t'][:,None]))
                    *np.array([0.,0.,D[0],D[1],0.,0.])/6))))


def large_angle_checks():
    a=np.array([2.,2.]); b=np.array([3.,3.])
    E=np.array([2*np.sin(np.deg2rad(150)/2),.4]); S=a*E
    D=.8*b*S; rows=[]
    for j in range(8):
        x0=np.r_[E[0]*unit(RNG.normal(size=3)),E[1]*unit(RNG.normal(size=3))]
        s0=np.r_[S[0]*unit(RNG.normal(size=3)),S[1]*unit(RNG.normal(size=3))]
        f=RNG.uniform(.1,2,2)
        def disturbance(t,x,s,u,f=f):
            return np.r_[D[0]*np.array([np.cos(f[0]*t),np.sin(f[0]*t),0]),
                         D[1]*np.array([0,np.cos(f[1]*t),np.sin(f[1]*t)])]
        sim=integrate(a,b,disturbance,make_state(x0,s0,a),duration=10.)
        rows.append(dict(run=j,peak_pose_ratio=(sim['nx']/E).max(axis=0).tolist(),
                         peak_filtered_ratio=(sim['ns']/S).max(axis=0).tolist(),
                         minimum_eta=float(sim['y'][:,0].min()),
                         quaternion_norm_error=sim['qnorm_error']))
    assert max(max(r['peak_pose_ratio']) for r in rows) < 1+2e-6
    assert min(r['minimum_eta'] for r in rows)>=np.cos(np.deg2rad(75))-2e-6
    return dict(angle_domain_deg=150,E=E.tolist(),D=D.tolist(),runs=rows)


def robust_design():
    ell, L, eps_p, eps_angle = 1., .5, .01, np.deg2rad(.5)
    E=np.array([2*np.sin(eps_angle/2), (eps_p-2*L*np.sin(eps_angle/2))/ell])
    a=np.array([3.,3.]); R=np.array([.12,.03])
    Gamma=np.array([[.05,.01],[.01,.05]])
    S=a*E
    C=np.array([2*a[0]**2*E[0],2*a[1]**2*E[1]+2*a[0]*a[1]*E[0]*E[1]])
    # 20% reserve on additive residual; b >= a is a design preference.
    lp=linprog(1/S,A_ub=-(np.eye(2)-Gamma),b_ub=-(Gamma@C+1.2*R),
               bounds=[(a[i]*S[i],None) for i in range(2)],method='highs')
    assert lp.success
    b=lp.x/S; U=lp.x+C; D=Gamma@U+R; slack=lp.x-D
    assert min(slack)>0
    rows=[]; plot_sim=None
    for j in range(24):
        direction=unit(RNG.normal(size=3))
        x0=np.r_[E[0]*direction,E[1]*unit(RNG.normal(size=3))]
        s0=np.r_[S[0]*unit(RNG.normal(size=3)),S[1]*unit(RNG.normal(size=3))]
        if j < 12:
            # Rank-one, state-varying blocks point uncertainty toward s.
            # Each block has spectral norm <= Gamma[i,j].
            def disturbance(t,x,s,u):
                di=[]
                for i in range(2):
                    si=s[3*i:3*i+3]; direction=si/(np.linalg.norm(si)+1e-12)
                    di.append(direction*(R[i]+sum(Gamma[i,k]*np.linalg.norm(u[3*k:3*k+3])
                                                   for k in range(2))))
                return np.r_[di[0],di[1]]
        else:
            Q=[[np.linalg.qr(RNG.normal(size=(3,3)))[0] for k in range(2)] for i in range(2)]
            freq=RNG.uniform(.2,5,(2,2)); axes=[unit(RNG.normal(size=3)) for i in range(2)]
            def disturbance(t,x,s,u,Q=Q,freq=freq,axes=axes):
                return np.concatenate([R[i]*np.cos((i+1)*t)*axes[i]+sum(
                    Gamma[i,k]*np.sin(freq[i,k]*t)*Q[i][k]@u[3*k:3*k+3] for k in range(2))
                    for i in range(2)])
        sim=integrate(a,b,disturbance,make_state(x0,s0,a))
        rows.append(dict(run=j,peak_pose_ratio=(sim['nx']/E).max(axis=0).tolist(),
                         peak_filtered_ratio=(sim['ns']/S).max(axis=0).tolist(),
                         quaternion_norm_error=sim['qnorm_error']))
        if j == 0: plot_sim=sim
    assert max(max(row['peak_pose_ratio']) for row in rows)<1+2e-6
    assert max(max(row['peak_filtered_ratio']) for row in rows)<1+2e-6
    fig,ax=plt.subplots(1,2,figsize=(10,3.5))
    for i,label in enumerate(['Rotation','Translation']):
        ax[i].plot(plot_sim['t'],plot_sim['nx'][:,i]/E[i],label='Pose / target')
        ax[i].plot(plot_sim['t'],plot_sim['ns'][:,i]/S[i],label='Filtered error / target')
        ax[i].axhline(1,color='k',ls='--'); ax[i].set(title=label,xlabel='Time (s)',ylabel='Ratio')
        ax[i].legend()
    fig.tight_layout(); fig.savefig(OUT/'robust_tube.png',dpi=170); plt.close(fig)
    return dict(E=E.tolist(),a=a.tolist(),b=b.tolist(),S=S.tolist(),Gamma=Gamma.tolist(),
                R=R.tolist(),C=C.tolist(),U=U.tolist(),D=D.tolist(),slack=slack.tolist(),
                equivalent_Kp=[float(4*a[0]*b[0]),float(a[1]*b[1])],
                equivalent_Kd=(a+b).tolist(),runs=rows)


def l2_nonlinear_check():
    a=np.array([3.,3.]); b=np.array([6.,6.]); duration=120.; w=2*np.pi/60
    def disturbance(t,x,s,u):
        amp=np.sin(w*t) if t <= duration else 0.
        return amp*np.array([.18,0,0,.09,0,0])
    sim=integrate(a,b,disturbance,make_state(np.zeros(6),np.zeros(6),a),duration+12,dt=.02)
    d=np.array([disturbance(t,None,None,None) for t in sim['t']])
    energy=lambda z: np.trapz(np.sum(z*z,axis=1),sim['t'])
    r=np.sqrt(energy(sim['x'])/energy(d))
    r_s=np.sqrt(energy(sim['s'])/energy(d))
    assert r <= 1/18+1e-6 and r_s <= 1/6+1e-6
    return dict(pose_ratio=float(r),pose_bound=1/18,filtered_ratio=float(r_s),filtered_bound=1/6,
                maximum_angle_deg=float(np.rad2deg(2*np.arcsin(np.max(sim['nx'][:,0])/2))))


def original_law_design(base):
    a=np.array(base['a']); E=np.array(base['E']); S=a*E
    Gamma=np.array(base['Gamma']); R=np.array(base['R'])
    eta0=np.sqrt(1-E[0]**2/4); q=1-eta0; h=np.sqrt(2*q)
    Q=np.diag([q,0.]); M=np.diag([1+q,1.]); C0=2*a*a*E
    g=np.array([2*a[0]**2*E[0]*h,2*a[0]*a[1]*E[0]*E[1]])
    N=Q+Gamma@M
    lp=linprog(1/S,A_ub=-(np.eye(2)-N),b_ub=-(Gamma@C0+1.2*R+g),
               bounds=[(a[i]*S[i],None) for i in range(2)],method='highs')
    assert lp.success
    b=lp.x/S; U=M@lp.x+C0; D=Gamma@U+R
    geom_bound=Q@lp.x+g; slack=lp.x-D-geom_bound
    rows=[]; identity_error=0.; geom_ratio=np.zeros(2); u_ratio=np.zeros(2)
    for j in range(1000):
        x=np.r_[E[0]*unit(RNG.normal(size=3))*RNG.uniform(),
                E[1]*unit(RNG.normal(size=3))*RNG.uniform()]
        s=np.r_[S[0]*unit(RNG.normal(size=3))*RNG.uniform(),
                S[1]*unit(RNG.normal(size=3))*RNG.uniform()]
        y=make_state(x,s,a); v=y[7:]; _,B=blocks(y[:4],y[4:7])
        aa,bb=np.repeat(a,3),np.repeat(b,3)
        uold=-(aa+bb)*v-aa*bb*x*np.r_[np.full(3,y[0]),np.ones(3)]
        delta=np.r_[a[0]*(B[:3,:3]-np.eye(3))@v[:3]+a[0]*b[0]*(1-y[0])*x[:3],
                    a[1]*np.cross(v[:3],x[3:])]
        identity_error=max(identity_error,float(max(abs(uold+aa*(B@v)+bb*s-delta))))
        geom_ratio=np.maximum(geom_ratio,norms(delta)/geom_bound)
        u_ratio=np.maximum(u_ratio,norms(uold)/U)
    assert identity_error<1e-11 and max(geom_ratio)<1 and max(u_ratio)<1
    for j in range(12):
        x0=np.r_[E[0]*unit(RNG.normal(size=3)),E[1]*unit(RNG.normal(size=3))]
        s0=np.r_[S[0]*unit(RNG.normal(size=3)),S[1]*unit(RNG.normal(size=3))]
        def disturbance(t,x,s,u):
            di=[]
            for i in range(2):
                si=s[3*i:3*i+3]; direction=si/(np.linalg.norm(si)+1e-12)
                di.append(direction*(R[i]+sum(Gamma[i,k]*np.linalg.norm(u[3*k:3*k+3])
                                               for k in range(2))))
            return np.r_[di[0],di[1]]
        sim=integrate(a,b,disturbance,make_state(x0,s0,a),law="original")
        rows.append(dict(run=j,peak_pose_ratio=(sim['nx']/E).max(axis=0).tolist(),
                         peak_filtered_ratio=(sim['ns']/S).max(axis=0).tolist(),
                         quaternion_norm_error=sim['qnorm_error']))
    assert max(max(z['peak_pose_ratio']) for z in rows)<1+2e-6
    assert max(max(z['peak_filtered_ratio']) for z in rows)<1+2e-6
    return dict(eta0=eta0,q=q,h=h,b=b.tolist(),U=U.tolist(),D=D.tolist(),
                geometric_bound=geom_bound.tolist(),slack=slack.tolist(),
                identity_error=identity_error,geometry_bound_utilization=geom_ratio.tolist(),
                control_bound_utilization=u_ratio.tolist(),runs=rows)


def counterexample_and_sampling(design):
    P=np.eye(2)+.5*np.array([[0.,-1.],[1.,0.]])
    F=np.block([[np.zeros((2,2)),np.eye(2)],[-10*P,-P]])
    eig=np.linalg.eigvals(F)
    assert eig.real.max()>0
    sampling=[]
    for a,b in zip(design['a'],design['b']):
        for h in [.01,.1,.25]:
            k,c=a*b,a+b
            Ap=np.array([[1.,h],[0.,1.]]); Bp=np.array([[h*h/2],[h]])
            K=np.array([[k,c]])
            F0=Ap-Bp@K
            F1=np.block([[Ap,-Bp@K],[np.eye(2),np.zeros((2,2))]])
            sampling.append(dict(a=a,b=b,h=h,rho_ZOH=float(max(abs(np.linalg.eigvals(F0)))),
                                 rho_one_step_delay=float(max(abs(np.linalg.eigvals(F1))))))
    return dict(counterexample_eigenvalues=[[float(z.real),float(z.imag)] for z in eig],
                alpha=.5,k=10.,damping=1.,sampling=sampling)


def main():
    report={'seed':20260919,'model':'offline continuous nonlinear pose/twist error model'}
    report['identities']=identity_checks()
    report['linear_frequency']=frequency_checks()
    report['additive']=additive_checks()
    report['robust']=robust_design()
    report['nonlinear_l2']=l2_nonlinear_check()
    report['counterexample_sampling']=counterexample_and_sampling(report['robust'])
    report['large_angle']=large_angle_checks()
    report['original_law']=original_law_design(report['robust'])
    (OUT/'results.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    summary={k:v for k,v in report.items() if k not in ['additive','robust','large_angle','original_law']}
    summary['constant_peak_bound_attainment']=report['additive']['constant_final_ratio']
    summary['robust_gains']={k:report['robust'][k] for k in ['E','a','b','equivalent_Kp','equivalent_Kd','slack']}
    summary['constant_filtered_solution_error']=report['additive']['constant_filtered_solution_error']
    summary['original_law_gains']=report['original_law']['b']
    summary['nonlinear_runs']=dict(additive=12,robust=24,large_angle=8,original_law=12,L2=1)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
