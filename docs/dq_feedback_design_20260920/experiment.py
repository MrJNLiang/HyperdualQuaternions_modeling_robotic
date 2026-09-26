"""Reproducible B601 offline rigid-body torque simulations (synthetic plants).
Run with Python >=3.8, numpy, scipy, pin (Pinocchio), matplotlib.
"""
from pathlib import Path
import argparse,csv,json,time,sys
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation
from controller import ROOT,Feedback
from core.dq_algebra import dq_from_r_p,dq_mul,vec6_to_pure_dq,dq_conj,dq_translation
from core.tndq_algebra import HDQ
from control.error_system import full_error_state

HERE=Path(__file__).resolve().parent
URDF=ROOT/'reBotArm_control_py/urdf/reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf'
Q0=np.array([.116,-1.292,-.889,-.329,-.362,.296])
ARMATURE=.01*np.ones(6)
LIMIT=np.array([27,27,27,7,7,7.])


def fk(model,data,q,v,a=None):
    pin.computeJointJacobiansTimeVariation(model,data,q,v)
    pin.updateFramePlacements(model,data)
    fid=model.getFrameId('end_link'); pose=data.oMf[fid]
    J=pin.getFrameJacobian(model,data,fid,pin.WORLD)[[3,4,5,0,1,2],:].copy()
    dJ=pin.getFrameJacobianTimeVariation(model,data,fid,pin.WORLD)[[3,4,5,0,1,2],:].copy()
    r=Rotation.from_matrix(pose.rotation).as_quat()[[3,0,1,2]]
    x=dq_from_r_p(r,pose.translation)
    xi=J@v; xd=.5*dq_mul(vec6_to_pure_dq(xi),x)
    return dict(x=x,x_breve=HDQ(x,xd),J=J,Jdot_qdot=dJ@v,
                xi=xi,xidot=dJ@v+J@(np.zeros(6) if a is None else a))


def reference(t,moving):
    if not moving:return Q0.copy(),np.zeros(6),np.zeros(6)
    amp=np.array([.12,.07,.08,.06,.07,.04]); w=.6
    # Periodic smooth trajectory; all laws start at the same reference state.
    return Q0+amp*np.sin(w*t),amp*w*np.cos(w*t),-amp*w*w*np.sin(w*t)


def prepare(case,T,dt):
    model=pin.buildModelFromUrdf(str(URDF));data=model.createData()
    moving=case in ('slow_tracking','friction_tracking','noise_tracking')
    return [fk(model,data,*reference(i*dt,moving)) for i in range(round(T/dt))]


def run(law,case,seed,desired,T=16.,dt=.005,substeps=5,scale=1.,ki=16.,harmonics=(.6,1.2,1.8),harmonic_gain=16.):
    nominal=pin.buildModelFromUrdf(str(URDF)); plant=pin.buildModelFromUrdf(str(URDF))
    rng=np.random.default_rng(seed)
    mass_scale=rng.uniform(.9,1.1,6)
    mismatch=case in ('model_mismatch','friction_tracking','saturation_stress')
    if mismatch:
        for j in range(1,plant.njoints):
            inertia=plant.inertias[j]; factor=float(mass_scale[j-1])
            plant.inertias[j]=pin.Inertia(inertia.mass*factor,inertia.lever.copy(),inertia.inertia.copy()*factor)
    actual_arm=ARMATURE*(1.25 if mismatch else 1.)
    nd=nominal.createData();fd=nominal.createData();pd=plant.createData();tf=plant.createData()
    moving=case in ('slow_tracking','friction_tracking','noise_tracking')
    q,v,_=reference(0,moving)
    if case=='clean':q=q+np.array([.08,-.05,.06,.04,-.03,.05])
    ctrl=Feedback(law,ki=ki,scale=scale,harmonics=harmonics,harmonic_gain=harmonic_gain)
    bias=np.array([.12,-.18,.14,-.035,.025,-.02])*rng.uniform(.8,1.2,6)
    noise=case in ('noise_tracking','saturation_stress')
    sq,sv=(5e-5,1e-3) if noise else (0.,0.)
    out=[];prev_tau=None;failed=False;min_sigma=1.
    h=dt/substeps
    for k,des in enumerate(desired):
        t=k*dt
        qm=q+sq*rng.standard_normal(6);vm=v+sv*rng.standard_normal(6)
        current=fk(nominal,fd,qm,vm)
        err=full_error_state(current['x_breve'],des['x_breve'])
        cmd,s,ell=ctrl.command(err,des['xi'],des['xidot'])
        J=current['J'];sig=np.linalg.svd(J,compute_uv=False)[-1];min_sigma=min(min_sigma,sig)
        accel=np.linalg.solve(J@J.T+1e-8*np.eye(6),cmd-current['Jdot_qdot'])
        accel=J.T@accel
        amax=3. if case=='saturation_stress' else 12.
        clipped=np.any(np.abs(accel)>amax);accel=np.clip(accel,-amax,amax)
        M=pin.crba(nominal,nd,qm).copy()+np.diag(ARMATURE)
        nle=pin.nonLinearEffects(nominal,nd,qm,vm).copy()
        raw=M@accel+nle;tau=np.clip(raw,-LIMIT,LIMIT)
        clipped=bool(clipped or np.any(tau!=raw))
        if prev_tau is not None:
            limited=prev_tau+np.clip(tau-prev_tau,-80*dt,80*dt)
            clipped=bool(clipped or np.max(np.abs(limited-tau))>1e-10)
            tau=limited
        ctrl.advance(s,ell,dt,limited=clipped)
        exact=fk(plant,tf,q,v)
        xerr=dq_mul(exact['x'],dq_conj(des['x']))
        angle=2*np.arctan2(np.linalg.norm(xerr[1:4]),abs(xerr[0]))
        pe=np.linalg.norm(dq_translation(exact['x'])-dq_translation(des['x']))
        out.append(np.r_[t,pe,angle,np.linalg.norm(dq_translation(xerr)),sig,int(clipped),q,v,tau,ctrl.bias])
        for sub in range(substeps):
            ts=t+sub*h
            Mtrue=pin.crba(plant,pd,q).copy()+np.diag(actual_arm)
            ht=pin.nonLinearEffects(plant,pd,q,v).copy()
            ext=np.zeros(6)
            if case in ('constant_bias','slow_tracking','model_mismatch','saturation_stress'):
                ext=bias*min(1.,max(0.,(ts-2.)/.2))
            if case=='slow_tracking':ext*=1+.2*np.sin(.4*ts)
            if case in ('pulse','pulse_small'):
                ext=bias*(4 if case=='pulse' else 1) if 3<=ts<3.4 else np.zeros(6)
            fric=np.zeros(6)
            if case=='friction_tracking':fric=.12*v+np.array([.12,.2,.18,.07,.05,.04])*np.tanh(v/.02)
            acc=np.linalg.solve(Mtrue,tau+ext-ht-fric)
            v=v+h*acc;q=q+h*v
        prev_tau=tau.copy()
        if (not np.all(np.isfinite(q)) or np.max(np.abs(v))>10 or
            np.any(q<plant.lowerPositionLimit) or np.any(q>plant.upperPositionLimit)):
            failed=True;break
    z=np.array(out);late=z[:,0]>=T*.7
    if not late.any():late=np.ones(len(z),bool)
    tau=z[:,18:24];vlog=z[:,12:18]
    rms=lambda a:float(np.sqrt(np.mean(np.square(a))))
    metrics=dict(law=law,case=case,seed=seed,scale=scale,ki=ki,dt=dt,
      pos_rms_mm=1000*rms(z[late,1]),angle_rms_deg=180/np.pi*rms(z[late,2]),
      pos_all_mm=1000*rms(z[:,1]),angle_all_deg=180/np.pi*rms(z[:,2]),
      torque_rms=float(np.sqrt(np.mean(np.sum(tau*tau,axis=1)))),
      torque_peak=float(np.max(np.abs(tau))),
      torque_rate_rms=float(np.sqrt(np.mean(np.sum(np.diff(tau,axis=0)**2,axis=1))))/dt,
      jerk_rms=float(np.sqrt(np.mean(np.sum(np.diff(vlog,n=2,axis=0)**2,axis=1))))/dt**2,
      limited_fraction=float(np.mean(z[:,5])),min_sigma=float(min_sigma),failed=failed,
      simulated_seconds=float(z[-1,0]+dt),metrics_scope='incomplete_prefix' if failed else 'full_run',
      bias_final=float(np.linalg.norm(ctrl.bias)))
    return metrics,z


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--quick',action='store_true');ap.add_argument('--out',default=str(HERE/'results'))
    ap.add_argument('--dt',type=float,default=.005);ap.add_argument('--seeds',type=int,default=3)
    ap.add_argument('--laws',nargs='+');ap.add_argument('--cases',nargs='+');ap.add_argument('--T',type=float,default=16.)
    ap.add_argument('--ki',type=float,default=16.)
    args=ap.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    cases=['clean','constant_bias','slow_tracking','model_mismatch','friction_tracking','noise_tracking','pulse','saturation_stress']
    laws=['C1','C2','DQ-noI','DQ-I','C2+I']
    if args.quick:cases=['clean','constant_bias','friction_tracking'];laws=['C2','DQ-I','C2+I']
    if args.laws:laws=args.laws
    if args.cases:cases=args.cases
    allrows=[]
    for case in cases:
        desired=prepare(case,args.T,args.dt)
        for seed in range(args.seeds):
            for law in laws:
                m,z=run(law,case,seed,desired,T=args.T,dt=args.dt,ki=args.ki)
                allrows.append(m)
                if seed==0:np.savez_compressed(out/(case+'_'+law.replace('+','plus')+'.npz'),trace=z)
                print(json.dumps(m),flush=True)
        with (out/'metrics.json').open('w') as f:json.dump(allrows,f,indent=2)
    with (out/'metrics.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=allrows[0].keys());w.writeheader();w.writerows(allrows)
    manifest=dict(urdf=str(URDF.relative_to(ROOT)),T=args.T,dt=args.dt,substeps=5,
      nominal_armature=ARMATURE.tolist(),nominal_d=15,nominal_k=36,lambda_=3,kappa=12,ki=args.ki,
      harmonics=[.6,1.2,1.8],harmonic_gain=16.,cases=cases,laws=laws,
      limits=LIMIT.tolist(),seeds=args.seeds,plant='URDF rigid body; synthetic uncertainties, no MIT tether, no hardware contact',
      integration='semi-implicit Euler, five substeps; sampled controller, ZOH torque',
      python=sys.version,pinocchio=pin.__version__,numpy=np.__version__)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
if __name__=='__main__':main()
