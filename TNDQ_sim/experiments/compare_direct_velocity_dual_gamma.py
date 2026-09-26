"""Direct velocity integration sweep for dual-channel screw-log H-infinity gains."""
import csv, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import params
from core.kinematics import TNDQSerialChain
from control.error_system import full_error_state
from control.control_law import dq_hinf_kinematic_law, dq_log_hinf_kinematic_law, damped_pinv
from simdata.trajectory_generator import LineTrajectoryTNDQ

DT, T_END = .01, 5.0
GAMMA_O, GAMMA_T = np.sqrt(2.)/8., np.sqrt(2.)/4.
ROOT = os.path.join(os.path.dirname(__file__), "..", "results")

def simulate(kind, scale, gamma_R=None, gamma_T=None):
    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    q = (params.Q_INIT + scale*np.array([1,-1,1,-1,1,-1,1])).copy()
    target = chain.fkm(params.Q_INIT)
    traj = LineTrajectoryTNDQ(target, [.15,.10,-.10], 3.5, [0,0,1], .5)
    p, th, cmd, dcmd = [], [], [], []
    prev = None
    for i in range(round(T_END/DT)):
        fk = chain.fk_outputs(q, np.zeros(chain.n), with_jacobian=True)
        desired = traj.evaluate(i*DT)
        err = full_error_state(fk["x_breve"], desired["x_breve_d"])
        if kind == "FA":
            task = dq_hinf_kinematic_law(err, desired["xi_d"], GAMMA_O, GAMMA_T)
        else:
            task = dq_log_hinf_kinematic_law(err, desired["xi_d"], gamma_R, gamma_T)
        cmd_i = damped_pinv(fk["J"], damping=max(params.PINV_DAMPING, params.DQH_DAMPING)) @ task
        q = q + DT*cmd_i
        p.append(np.linalg.norm(err["T"]))
        th.append(2*np.arctan2(np.linalg.norm(err["x_tilde"][1:4]), max(0.,err["x_tilde"][0])))
        cmd.append(cmd_i.copy())
        if prev is not None: dcmd.append((cmd_i-prev)/DT)
        prev=cmd_i
    p,th,cmd,dcmd=map(np.asarray,(p,th,cmd,dcmd))
    def sustained(deg):
        ok=th<np.deg2rad(deg); z=np.logical_and.accumulate(ok[::-1])[::-1]; ix=np.where(z)[0]
        return float(ix[0]*DT) if len(ix) else np.nan
    return dict(law=kind, scale=scale, gamma_R=gamma_R, gamma_T=gamma_T,
        cmd_rms=float(np.sqrt(np.mean(np.sum(cmd**2,axis=1)))),
        cmd_peak=float(np.max(np.linalg.norm(cmd,axis=1))),
        input_energy=float(DT*np.sum(np.sum(cmd**2,axis=1))),
        p_int=float(DT*np.sum(p)), p_rms=float(np.sqrt(np.mean(p**2))),
        theta_int_deg_s=float(np.rad2deg(DT*np.sum(th))),
        theta_rms_deg=float(np.rad2deg(np.sqrt(np.mean(th**2)))),
        t5_s=sustained(5), t2_s=sustained(2),
        joint_speed_peak=float(np.max(np.abs(cmd))),
        joint_jitter_rms=float(np.sqrt(np.mean(dcmd**2))),
        joint_jitter_peak=float(np.max(np.abs(dcmd))))

def main():
    rows=[]
    # For each channel ratio, scale both gammas until total RMS matches FA.
    ratios=(0.5, 1.0, 2.0, 4.0)  # gamma_T/gamma_R
    for scale in (.05,.08,.12):
        base=simulate("FA",scale); target=base["cmd_rms"]
        for ratio in ratios:
            lo,hi=.03,1.5
            for _ in range(9):
                gR=(lo+hi)/2; gT=ratio*gR
                r=simulate("log",scale,gR,gT)
                # smaller gamma means larger feedback and larger velocity
                if r["cmd_rms"] > target: lo=gR
                else: hi=gR
            gR=(lo+hi)/2; gT=ratio*gR
            r=simulate("log",scale,gR,gT)
            r["ratio_T_R"]=ratio; r["rms_match_pct"]=100*(r["cmd_rms"]-target)/target
            rows += [dict(law="FA",scale=scale,gamma_R=GAMMA_O,gamma_T=GAMMA_T,ratio_T_R=GAMMA_T/GAMMA_O,rms_match_pct=0,**{k:v for k,v in base.items() if k not in ("law","scale","gamma_R","gamma_T")}),r]
            print(scale,ratio,gR,gT,r["cmd_rms"],r["theta_int_deg_s"],r["p_int"],r["joint_jitter_rms"])
    path=os.path.join(ROOT,"direct_velocity_dual_gamma_sweep.csv"); os.makedirs(ROOT,exist_ok=True)
    with open(path,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    print(path)
if __name__=="__main__": main()
