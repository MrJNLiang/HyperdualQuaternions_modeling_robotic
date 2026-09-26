"""Direct qdot integration comparison under a matched RMS joint-speed budget."""
import csv, os, sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import params
from core.kinematics import TNDQSerialChain
from control.error_system import full_error_state
from control.control_law import (dq_hinf_kinematic_law,
    dq_log_structured_kinematic_law, damped_pinv)
from simdata.trajectory_generator import LineTrajectoryTNDQ

DT, T_END = 0.01, 5.0
GAMMA_O, GAMMA_T = np.sqrt(2.)/8., np.sqrt(2.)/4.
ROOT = os.path.join(os.path.dirname(__file__), "..", "results")

def simulate(kind, scale, alpha=1.0, noisy=False):
    chain = TNDQSerialChain(params.KUKA_LBR4_DH)
    q = (params.Q_INIT + scale*np.array([1,-1,1,-1,1,-1,1])).copy()
    target = chain.fkm(params.Q_INIT)
    traj = LineTrajectoryTNDQ(target, [.15,.10,-.10], 3.5, [0,0,1], .5)
    rng = np.random.default_rng(271828)
    previous = None
    p, th, cmd, qhist, dcmd = [], [], [], [], []
    for i in range(round(T_END/DT)):
        t = i*DT
        qm = q + rng.normal(0., params.NOISE_SIGMA_Q, chain.n) if noisy else q
        fk = chain.fk_outputs(qm, np.zeros(chain.n), with_jacobian=True)
        desired = traj.evaluate(t)
        err = full_error_state(fk["x_breve"], desired["x_breve_d"])
        if kind == "FA":
            task = dq_hinf_kinematic_law(err, desired["xi_d"], GAMMA_O, GAMMA_T)
        else:
            task = dq_log_structured_kinematic_law(
                err, desired["xi_d"], np.diag([4*alpha]*6))
        cmd_i = damped_pinv(fk["J"], damping=max(params.PINV_DAMPING,
                    params.DQH_DAMPING)) @ task
        q = q + DT*cmd_i
        p.append(np.linalg.norm(err["T"]))
        th.append(2*np.arctan2(np.linalg.norm(err["x_tilde"][1:4]),
                               max(0., err["x_tilde"][0])))
        cmd.append(cmd_i.copy()); qhist.append(q.copy())
        if previous is not None: dcmd.append((cmd_i-previous)/DT)
        previous = cmd_i
    p, th, cmd, dcmd = map(np.asarray, (p,th,cmd,dcmd))
    def sustained(deg):
        ok = th < np.deg2rad(deg)
        z = np.logical_and.accumulate(ok[::-1])[::-1]; ix=np.where(z)[0]
        return float(ix[0]*DT) if len(ix) else np.nan
    return dict(law=kind, scale=scale, alpha=alpha, noisy=noisy,
        cmd_rms=float(np.sqrt(np.mean(np.sum(cmd**2,axis=1)))),
        cmd_peak=float(np.max(np.linalg.norm(cmd,axis=1))),
        input_energy=float(DT*np.sum(np.sum(cmd**2,axis=1))),
        p_rms=float(np.sqrt(np.mean(p**2))), p_mean=float(np.mean(p)),
        p_int=float(DT*np.sum(p)), p_end=float(p[-1]), p_peak=float(np.max(p)),
        theta_rms_deg=float(np.rad2deg(np.sqrt(np.mean(th**2)))),
        theta_mean_deg=float(np.rad2deg(np.mean(th))),
        theta_int_deg_s=float(np.rad2deg(DT*np.sum(th))),
        theta_end_deg=float(np.rad2deg(th[-1])),
        theta_peak_deg=float(np.rad2deg(np.max(th))),
        t10_s=sustained(10), t5_s=sustained(5), t2_s=sustained(2),
        joint_speed_rms=float(np.sqrt(np.mean(cmd**2))),
        joint_speed_peak=float(np.max(np.abs(cmd))),
        joint_jitter_rms=float(np.sqrt(np.mean(dcmd**2))),
        joint_jitter_peak=float(np.max(np.abs(dcmd))))

def match_alpha(scale, noisy=False, target=None):
    if target is None: target=simulate("FA",scale,noisy=noisy)["cmd_rms"]
    lo, hi=.1, 3.
    # Seven iterations keep the RMS matching error below about 1 percent
    # while avoiding dozens of full forward-kinematics runs per case.
    for _ in range(7):
        mid=(lo+hi)/2
        if simulate("log",scale,mid,noisy)["cmd_rms"] < target: lo=mid
        else: hi=mid
    return (lo+hi)/2

def main():
    rows=[]
    for scale in (.05,.08,.12):
        base=simulate("FA",scale)
        alpha=match_alpha(scale,target=base["cmd_rms"])
        rows += [base, simulate("log",scale,alpha)]
        print("scale",scale,"initial-angle-deg",base["theta_peak_deg"],
              "matched-alpha",alpha)
        for r in rows[-2:]: print(r)
    path=os.path.join(ROOT,"direct_velocity_rms_comparison.csv")
    os.makedirs(ROOT,exist_ok=True)
    with open(path,"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    print("results",path)

if __name__ == "__main__": main()
