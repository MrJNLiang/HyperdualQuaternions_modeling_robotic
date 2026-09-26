"""Dissipation and rejection checks for the harmonic DQ internal model."""
import json
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from controller import Feedback
from controller import pose_geometry
from core.dq_algebra import dq_from_r_p,q_mul
from verify import OUT

LAM,KAPPA,K0,KJ=3.,12.,16.,16.
NU=np.array([.6,1.2,1.8])

def discrete_oscillator_check():
    rng=np.random.default_rng(24092026);maximum=0.
    for _ in range(100):
        ctrl=Feedback('DQ-IM')
        ctrl.osc_a=rng.normal(size=(3,6));ctrl.osc_c=rng.normal(size=(3,6))
        a0=ctrl.osc_a.copy();c0=ctrl.osc_c.copy()
        s=rng.normal(size=6);dt=rng.uniform(.001,.02);limited=bool(rng.integers(2))
        ctrl.advance(s,np.zeros(6),dt,limited=limited)
        for j,nu in enumerate(NU):
            generator=np.array([[0.,nu,1.],[-nu,0.,0.],[0.,0.,0.]])
            held_input=np.zeros(6) if limited else KJ*s
            exact=expm(dt*generator)@np.vstack((a0[j],c0[j],held_input))
            maximum=max(maximum,float(np.max(np.abs(exact[:2]-np.vstack((ctrl.osc_a[j],ctrl.osc_c[j]))))))
    assert maximum<1e-12,maximum
    return maximum

def true_model(t,amplitude):
    a=np.array([[.12,-.08,.06,.05,-.02,.04],[.03,.02,-.03,.04,.01,-.02],[.02,-.04,.01,.02,.03,-.01]])*amplitude
    c=np.roll(a,1,axis=1)*.7
    ct=np.cos(NU*t)[:,None];st=np.sin(NU*t)[:,None]
    return ct*a+st*c,-st*a+ct*c

def run(gamma,amplitude):
    kap=1/(LAM*gamma)
    b=np.array([.1,-.05,.04,.03,.01,-.02])*amplitude
    def rhs(t,y):
        r,p,e,bhat=y[:4],y[4:7],y[7:13],y[13:19]
        ah=y[19:37].reshape(3,6);ch=y[37:55].reshape(3,6)
        a,c=true_model(t,amplitude)
        w=np.sin(np.pi*t/12)**2*np.array([.4*np.sin(t),.3*np.cos(2*t),.2*np.sin(.7*t),.1*np.cos(t),.3*np.sin(1.8*t),.2*np.cos(.3*t)]) if t<12 else np.zeros(6)
        h,B,U=pose_geometry(dq_from_r_p(r,p));s=e+LAM*h
        dr=.5*q_mul(np.r_[0,e[:3]],r);dp=np.cross(e[:3],p)+e[3:]
        de=-LAM*B@e-kap*s-bhat-ah.sum(axis=0)+b+a.sum(axis=0)+w
        return np.r_[dr,dp,de,K0*s,(NU[:,None]*ch+KJ*s).ravel(),(-NU[:,None]*ah).ravel(),h@h,w@w]
    y0=np.zeros(57);y0[0]=1.
    sol=solve_ivp(rhs,(0,120),y0,rtol=2e-10,atol=2e-12,max_step=.04,dense_output=True)
    assert sol.success
    t=np.linspace(0,120,6001);y=sol.sol(t);hh=[];Vs=[]
    for i,tt in enumerate(t):
        h,B,U=pose_geometry(dq_from_r_p(y[:4,i],y[4:7,i]));s=y[7:13,i]+LAM*h
        a,c=true_model(tt,amplitude)
        da=y[19:37,i].reshape(3,6)-a;dc=y[37:55,i].reshape(3,6)-c
        W=.5*(s@s)+np.sum((y[13:19,i]-b)**2)/(2*K0)+(np.sum(da**2)+np.sum(dc**2))/(2*KJ)
        Vs.append(U/LAM+W/(LAM*LAM*kap));hh.append(h)
    V=np.asarray(Vs);h=np.asarray(hh);margin=2*V[0]+gamma**2*y[56]-y[55]-2*V
    result=dict(gamma=gamma,modeled_disturbance=bool(amplitude),min_eta=float(y[0].min()),
      min_integrated_dissipation_margin=float(margin.min()),
      energy_output=float(y[55,-1]),energy_bound=float(2*V[0]+gamma**2*y[56,-1]),
      final_h_norm=float(np.linalg.norm(h[-1])),
      late_h_rms=float(np.sqrt(np.mean(np.sum(h[t>=100]**2,axis=1)))),
      induced_ratio=float(np.sqrt(y[55,-1]/y[56,-1])) if not amplitude else None)
    assert margin.min()>-1e-7,result
    np.savez_compressed(OUT/('im_certificate_'+str(round(gamma,5))+'_'+str(amplitude)+'.npz'),t=t,h=h,margin=margin,V=V,input_energy=y[56],output_energy=y[55])
    return result

def main():
    rr=[run(g,amp) for g,amp in ((1/36,1),(.02,0),(1/36,0),(.05,0))]
    om=np.sort(np.r_[np.logspace(-5,3,40000),NU*(1+1e-9)]);z=1j*om
    K=K0/z+sum(KJ*z/(z*z+nu*nu) for nu in NU)
    G=1/((z+LAM)*(z+KAPPA+K));C2=1/((z+LAM)*(z+KAPPA))
    freq=dict(hinf_dense_grid=float(np.abs(G).max()),bound=1/(LAM*KAPPA),max_ratio_to_C2=float(np.abs(G/C2).max()),
              rejection_near_selected_frequencies=[float(np.abs(G[np.argmin(np.abs(om-nu))])) for nu in NU])
    np.savez_compressed(OUT/'frequency_im.npz',omega=om,candidate=np.abs(G),ratio=np.abs(G/C2))
    result=dict(nonlinear=rr,frequency=freq,discrete_oscillator_error=discrete_oscillator_check())
    (OUT/'verification_im.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
