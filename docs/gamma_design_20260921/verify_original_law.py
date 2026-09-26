"""Validate peak-budget design while retaining the original A^T Kp ez law.
Offline nonlinear error system; no hardware or plant-identification claims.
"""
from pathlib import Path
import sys,json
import numpy as np
from scipy.integrate import solve_ivp
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'TNDQ_sim'))
from core.dq_algebra import dq_from_r_p,skew,q_mul
from control.error_system import A_matrix,output_error
RNG=np.random.default_rng(20260921)

def unit(z):return z/np.linalg.norm(z)
def params(theta_deg,ET,DR,DT,margin=1.05):
    ER=2*np.sin(np.deg2rad(theta_deg)/2);eta=np.cos(np.deg2rad(theta_deg)/2)
    a=np.array([3.,3.]);D=np.array([DR,DT]);E=np.array([ER,ET])
    b=np.array([DR/(a[0]*eta*ER),2*a[0]*ER+DT/(a[1]*ET)])*margin
    return a,b,D,E,eta

def calc(y,a,b):
    r,p,v=y[:4],y[4:7],y[7:13]
    x=np.r_[2*r[1:],p];aa=np.repeat(a,3);bb=np.repeat(b,3)
    B=np.block([[r[0]*np.eye(3)-skew(r[1:]),np.zeros((3,3))],[-skew(p),np.eye(3)]])
    xe=dq_from_r_p(r,p);A=A_matrix(xe);ez,_,_=output_error(xe)
    kp=np.diag(np.r_[np.full(3,4*a[0]*b[0]),np.full(3,a[1]*b[1])])
    u=-(aa+bb)*v-A.T@kp@ez
    return x,B,u,v+aa*x

def make_state(E,a,on_boundary=True):
    x=np.r_[unit(RNG.normal(size=3))*E[0],unit(RNG.normal(size=3))*E[1]]
    if not on_boundary:x*=np.repeat(RNG.uniform(0,1,2),3)
    s=np.r_[unit(RNG.normal(size=3))*a[0]*E[0],unit(RNG.normal(size=3))*a[1]*E[1]]
    return np.r_[np.sqrt(1-x[:3]@x[:3]/4),x[:3]/2,x[3:],s-np.repeat(a,3)*x]

def verify_identities():
    maxerr=0.; maxboundary=-np.inf
    for angle in (5,60,150):
        a,b,D,E,eta0=params(angle,.025,.5,.09,margin=1.)
        for _ in range(400):
            y=make_state(E,a,on_boundary=False);x,B,u,s=calc(y,a,b);v=y[7:]
            dp=np.r_[unit(RNG.normal(size=3))*D[0],unit(RNG.normal(size=3))*D[1]]
            ds=u+dp+np.repeat(a,3)*(B@v)
            eta=y[0];beta=1-eta
            expectedR=-(b[0]+a[0]*beta)*s[:3]-a[0]*np.cross(y[1:4],s[:3])+a[0]*(a[0]+b[0])*beta*x[:3]+dp[:3]
            expectedT=-b[1]*s[3:]+a[1]*np.cross(v[:3],x[3:])+dp[3:]
            maxerr=max(maxerr,float(np.max(np.abs(ds-np.r_[expectedR,expectedT]))))
            # The sampling constructs s on its tube boundary; choose worst
            # radial disturbance, independent of the identity input above.
            dw=np.r_[D[0]*unit(s[:3]),D[1]*unit(s[3:])]
            ds=u+dw+np.repeat(a,3)*(B@v)
            maxboundary=max(maxboundary,float(unit(s[:3])@ds[:3]),float(unit(s[3:])@ds[3:]))
    assert maxerr<1e-10 and maxboundary<1e-8,(maxerr,maxboundary)
    return dict(samples=1200,filtered_identity_error=maxerr,max_sampled_boundary_derivative=maxboundary)

def run(angle,mode,seed):
    a,b,D,E,eta0=params(angle,.025,.5,.09)
    gamma_matrix=np.array([[.04,.015],[.01,.04]]) if mode=='multiplicative' else np.zeros((2,2))
    residual=D.copy()
    if mode=='multiplicative':
        C=2*a*a*E;H=np.diag([eta0,1.]);M=np.diag([2-eta0,1.])
        g=np.array([0.,2*a[1]*a[0]*E[0]*E[1]])
        t=np.linalg.solve(H-gamma_matrix@M,gamma_matrix@C+residual+g)*1.05
        assert np.all((H-gamma_matrix@M)@t >=gamma_matrix@C+residual+g-1e-12)
        b=t/(a*E)
    if mode=='constant':y0=np.r_[1.,np.zeros(12)]
    else:y0=make_state(E,a)
    local=np.random.default_rng(seed)
    phases=local.uniform(0,6,2);freq=local.uniform(.5,4,2)
    def rhs(t,y):
        x,B,u,s=calc(y,a,b);v=y[7:];r=y[:4]
        if mode=='constant':d=np.array([D[0],0,0,D[1],0,0])
        elif mode=='rotating':
            d=np.r_[D[0]*np.array([np.cos(freq[0]*t+phases[0]),np.sin(freq[0]*t+phases[0]),0]),D[1]*np.array([np.cos(freq[1]*t+phases[1]),0,np.sin(freq[1]*t+phases[1])])]
        else:
            budget=gamma_matrix@np.array([np.linalg.norm(u[:3]),np.linalg.norm(u[3:])])+residual
            # Rank-one, state-dependent worst-direction block errors exist
            # with these operator norms; use smooth radial directions.
            d=np.r_[budget[0]*s[:3]/np.sqrt(s[:3]@s[:3]+1e-8),budget[1]*s[3:]/np.sqrt(s[3:]@s[3:]+1e-8)]
        return np.r_[.5*q_mul(np.r_[0.,v[:3]],r),(B@v)[3:],u+d]
    t=np.linspace(0,12,1201)
    sol=solve_ivp(rhs,(0,12),y0,t_eval=t,rtol=1e-9,atol=1e-11,max_step=.025)
    assert sol.success
    x=np.vstack((2*sol.y[1:4],sol.y[4:7]));s=sol.y[7:]+np.repeat(a,3)[:,None]*x
    ratios=np.vstack((np.linalg.norm(x[:3],axis=0)/E[0],np.linalg.norm(x[3:],axis=0)/E[1],
        np.linalg.norm(s[:3],axis=0)/(a[0]*E[0]),np.linalg.norm(s[3:],axis=0)/(a[1]*E[1])))
    ans=dict(theta_domain_deg=angle,mode=mode,seed=seed,a=a.tolist(),b=b.tolist(),E=E.tolist(),D=D.tolist(),
        max_ratios=ratios.max(axis=1).tolist(),minimum_eta=float(sol.y[0].min()),
        quaternion_norm_error=float(np.max(abs(np.linalg.norm(sol.y[:4],axis=0)-1))))
    assert ratios.max()<=1+2e-6,ans
    return ans

result=dict(identities=verify_identities(),runs=[])
for angle in (5,60,150):
    for mode in ('constant','rotating','multiplicative'):
        for seed in range(2):result['runs'].append(run(angle,mode,seed))
result['summary']=dict(runs=len(result['runs']),max_ratio=max(max(r['max_ratios']) for r in result['runs']),
                      max_quaternion_norm_error=max(r['quaternion_norm_error'] for r in result['runs']))
(HERE/'verification.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result['identities']));print(json.dumps(result['summary']))
