"""Offline DQ/HDQ feedback research. No actuator imports or hardware writes.

All translation is normalized with length ell=1 m in the supplied experiments.
C2 gains d=15, k=36; the candidate has lambda=3, kappa=12, ki=16.
The integral-disabled candidate has the same local PD gains as C2.
"""
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'TNDQ_sim'))
from core.dq_algebra import (dq_translation, dq_log2_vec6, skew)
from control.control_law import feedforward_term


def pose_geometry(x):
    eta = float(x[0]); mu = np.asarray(x[1:4]); p = dq_translation(x)
    h = np.r_[2*mu, p]
    B = np.block([[eta*np.eye(3)-skew(mu), np.zeros((3,3))],
                  [-skew(p), np.eye(3)]])
    U = 4*(1-eta) + .5*np.dot(p,p)
    return h, B, U


class Feedback:
    def __init__(self, law='DQ-I', lam=3., kappa=12., ki=16., scale=1., harmonics=(.6,1.2,1.8), harmonic_gain=16.):
        self.law=law; self.lam=lam; self.kappa=kappa; self.ki=ki
        self.d=15*np.sqrt(scale); self.k=36*scale
        self.bias=np.zeros(6); self.integral=np.zeros(6)
        self.frequencies=np.asarray(harmonics)
        self.harmonic_gain=harmonic_gain
        self.osc_a=np.zeros((len(harmonics),6)); self.osc_c=np.zeros_like(self.osc_a)
        self.initialized=False

    def command(self, err, xi_d, xidot_d):
        e=err['e_xi']; h,B,_=pose_geometry(err['x_tilde'])
        s=e+self.lam*h
        ff=feedforward_term(err['x_tilde'],err['xi_tilde'],xi_d,xidot_d)
        ell=dq_log2_vec6(err['x_tilde'])
        if not self.initialized:
            # Match the local initial command of C2+I to DQ-I (bias initially 0).
            self.integral=-ell/self.lam
            self.initialized=True
        if self.law=='C2':
            fb=-self.d*e-self.k*ell
        elif self.law=='C1':
            kp=np.diag([4*self.k]*3+[self.k]*3)
            fb=-self.d*e-err['A'].T@kp@err['e_z']
        elif self.law=='C2+I':
            fb=-(self.lam+self.kappa)*e-(self.lam*self.kappa+self.ki)*ell-self.lam*self.ki*self.integral
        elif self.law in ('DQ-I','DQ-noI','DQ-IM'):
            fb=-self.lam*(B@e)-self.kappa*s
            if self.law in ('DQ-I','DQ-IM'): fb-=self.bias
            if self.law=='DQ-IM': fb-=self.osc_a.sum(axis=0)
        elif self.law=='C2+IM':
            s=e+self.lam*ell
            fb=-(self.lam+self.kappa)*e-self.lam*self.kappa*ell-self.bias-self.osc_a.sum(axis=0)
        else: raise ValueError(self.law)
        return ff+fb, s, ell

    def advance(self, s, ell, dt, limited=False):
        # Conditional integration is a practical antiwindup heuristic. The proof
        # applies only while this gate and all actuator constraints are inactive.
        if not limited:
            if self.law in ('DQ-I','DQ-IM','C2+IM'): self.bias+=dt*self.ki*s
            if self.law=='C2+I': self.integral+=dt*ell
        if self.law in ('DQ-IM','C2+IM'):
            # Exact ZOH oscillator flow. Keep rotating on saturation, but gate
            # its error-driven update; this gate is outside the continuous proof.
            nu=self.frequencies[:,None]; c=np.cos(nu*dt);sn=np.sin(nu*dt)
            a0=self.osc_a.copy(); c0=self.osc_c.copy()
            drive=0.*s if limited else self.harmonic_gain*s
            self.osc_a=c*a0+sn*c0+(sn/nu)*drive
            self.osc_c=-sn*a0+c*c0-((1-c)/nu)*drive
