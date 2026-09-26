"""Independent algebra, kinematic and nonlinear dissipation validation."""
from pathlib import Path
import json
import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import freqresp, TransferFunction
from controller import pose_geometry
from experiment import fk,URDF,Q0
import pinocchio as pin
from core.dq_algebra import (dq_from_r_p,q_exp_axis,q_mul,vec6_to_pure_dq,
    dq_mul,dq_conj,dq_vec6,dq_pose_normalize)

HERE=Path(__file__).resolve().parent
OUT=HERE/'results';OUT.mkdir(exist_ok=True)

def algebra():
    rng=np.random.default_rng(20260920)
    maxima={k:0. for k in ('hdot','Udot','sdot','Wdot','dissipation_violation','fk_xdot','fk_xidot','hdq_twist')}
    lam,kappa,ki=3.,12.,16.
    for i in range(500):
        axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
        r=q_exp_axis(axis,rng.uniform(0,2.8));p=rng.normal(size=3)*.2
        x=dq_from_r_p(r,p);e=rng.normal(size=6);s=e+lam*pose_geometry(x)[0]
        bhat=rng.normal(size=6);b=rng.normal(size=6);w=rng.normal(size=6)
        h,B,U=pose_geometry(x)
        dr=.5*q_mul(np.r_[0.,e[:3]],r)
        dp=np.cross(e[:3],p)+e[3:]
        eps=1e-6
        xp=dq_from_r_p((r+eps*dr)/np.linalg.norm(r+eps*dr),p+eps*dp)
        xm=dq_from_r_p((r-eps*dr)/np.linalg.norm(r-eps*dr),p-eps*dp)
        hp,_,Up=pose_geometry(xp);hm,_,Um=pose_geometry(xm)
        maxima['hdot']=max(maxima['hdot'],np.max(np.abs((hp-hm)/(2*eps)-B@e)))
        maxima['Udot']=max(maxima['Udot'],abs((Up-Um)/(2*eps)-h@e))
        edot=-lam*B@e-kappa*s-bhat+b+w
        sdot=edot+lam*B@e
        maxima['sdot']=max(maxima['sdot'],np.max(np.abs(sdot-(-kappa*s-bhat+b+w))))
        wd=s@sdot+(bhat-b)@(ki*s)/ki
        maxima['Wdot']=max(maxima['Wdot'],abs(wd+kappa*(s@s)-s@w))
        vd=h@e/lam+wd/(lam*lam*kappa)
        bound=-.5*(h@h)+.5*(w@w)/(lam*kappa)**2
        maxima['dissipation_violation']=max(maxima['dissipation_violation'],vd-bound)
    model=pin.buildModelFromUrdf(str(URDF));data=model.createData()
    for i in range(100):
        q=Q0+rng.uniform(-.1,.1,6);v=rng.normal(size=6)*.1;a=rng.normal(size=6)*.2
        c=fk(model,data,q,v,a);eps=2e-6
        cp=fk(model,data,q+eps*v+.5*eps*eps*a,v+eps*a)
        cm=fk(model,data,q-eps*v+.5*eps*eps*a,v-eps*a)
        # Quaternion sign branches must agree for a meaningful difference.
        for cc in (cp,cm):
            if cc['x'][:4]@c['x'][:4]<0:cc['x']=-cc['x']
        xd_fd=(cp['x']-cm['x'])/(2*eps)
        maxima['fk_xdot']=max(maxima['fk_xdot'],np.max(np.abs(xd_fd-c['x_breve'].ch[1])))
        maxima['fk_xidot']=max(maxima['fk_xidot'],np.max(np.abs((cp['xi']-cm['xi'])/(2*eps)-c['xidot'])))
        twist=dq_vec6(2*dq_mul(xd_fd,dq_conj(c['x'])))
        maxima['hdq_twist']=max(maxima['hdq_twist'],np.max(np.abs(twist-c['xi'])))
    assert max(maxima.values())<1e-6,maxima
    return maxima


def nonlinear(gamma,bias=False,large_initial=False):
    lam=3.;kappa=1/(lam*gamma);ki=16.
    b=np.array([.10,-.08,.05,.04,-.03,.02]) if bias else np.zeros(6)
    axis=np.array([1.,2.,-1.]);axis/=np.linalg.norm(axis)
    y0=np.zeros(4+3+6+6+2);y0[0]=1.
    if large_initial:
        y0[:4]=q_exp_axis(axis,2.6);y0[4:7]=[.2,-.15,.1]
    def disturbance(t):
        if t>=12:return np.zeros(6)
        envelope=np.sin(np.pi*t/12)**2
        return envelope*np.array([.7*np.sin(1.1*t),.5*np.cos(2.3*t),.4*np.sin(.8*t),
                                  .3*np.cos(1.4*t),.5*np.sin(2.1*t),.2*np.cos(.7*t)])
    def rhs(t,y):
        r,p,e,bhat=y[:4],y[4:7],y[7:13],y[13:19]
        h,B,U=pose_geometry(dq_from_r_p(r,p));s=e+lam*h;w=disturbance(t)
        rd=.5*q_mul(np.r_[0.,e[:3]],r);pd=np.cross(e[:3],p)+e[3:]
        ed=-lam*B@e-kappa*s-bhat+b+w
        return np.r_[rd,pd,ed,ki*s,h@h,w@w]
    sol=solve_ivp(rhs,(0,24),y0,rtol=2e-10,atol=2e-12,max_step=.025,dense_output=True)
    assert sol.success,sol.message
    t=np.linspace(0,24,2401);y=sol.sol(t)
    h=np.concatenate((2*y[1:4],y[4:7]),axis=0);s=y[7:13]+lam*h
    W=.5*np.sum(s*s,axis=0)+np.sum((y[13:19]-b[:,None])**2,axis=0)/(2*ki)
    U=4*(1-y[0])+.5*np.sum(y[4:7]**2,axis=0)
    V=U/lam+W/(lam*lam*kappa)
    margin=2*V[0]+gamma**2*y[20]-y[19]-2*V
    result=dict(gamma=gamma,kappa=kappa,ki=ki,bias=bias,large_initial=large_initial,
       min_eta=float(y[0].min()),quaternion_norm_error=float(np.max(np.abs(np.linalg.norm(y[:4],axis=0)-1))),
       min_integrated_dissipation_margin=float(margin.min()),
       lhs_energy=float(y[19,-1]),rhs_energy=float(gamma**2*y[20,-1]+2*V[0]),
       induced_ratio=float(np.sqrt(y[19,-1]/y[20,-1])) if not bias and not large_initial else None,
       final_h_norm=float(np.linalg.norm(h[:,-1])),final_bias_error=float(np.linalg.norm(y[13:19,-1]-b)))
    assert result['min_integrated_dissipation_margin']>-1e-7,result
    assert result['min_eta']>0,result
    return result,dict(t=t,h=h,V=V,margin=margin,input_energy=y[20],output_energy=y[19])


def frequency():
    lam,kappa,ki=3.,12.,16.
    omega=np.logspace(-5,4,30000);z=1j*omega
    old=1/((z+lam)*(z+kappa));new=z/((z+lam)*(z*z+kappa*z+ki))
    ratio=np.abs(new/old)
    ans=dict(C2_hinf=float(np.abs(old).max()),new_hinf=float(np.abs(new).max()),
       new_hinf_bound=1/(lam*kappa),omega_new_peak=float(omega[np.argmax(np.abs(new))]),
       crossover_rad_s=float(np.sqrt(ki/2)),max_new_over_C2=float(ratio.max()))
    np.savez_compressed(OUT/'frequency.npz',omega=omega,C2=np.abs(old),candidate=np.abs(new),ratio=ratio)
    return ans


def main():
    result=dict(algebra=algebra(),frequency=frequency(),nonlinear=[])
    for gamma in (.02,1/36,.05,.08):
        m,z=nonlinear(gamma);result['nonlinear'].append(m)
        np.savez_compressed(OUT/('certificate_gamma_'+str(round(gamma,5))+'.npz'),**z)
    for bias,large in ((True,False),(False,True)):
        m,z=nonlinear(1/36,bias=bias,large_initial=large);result['nonlinear'].append(m)
    (OUT/'verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
