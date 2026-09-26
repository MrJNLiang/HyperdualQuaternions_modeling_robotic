"""Predeclared follow-up checks: gain/effort, frequency error, time resolution."""
from pathlib import Path
import json
import numpy as np
from experiment import prepare,run
OUT=Path(__file__).resolve().parent/'sensitivity_results';OUT.mkdir(exist_ok=True)
rows=[]
def trial(tag,law,case,seed=0,T=16.,dt=.005,**kw):
    d=prepare(case,T,dt);m,z=run(law,case,seed,d,T=T,dt=dt,**kw)
    m.update(tag=tag,substeps=kw.get('substeps',5),harmonics=list(kw.get('harmonics',(.6,1.2,1.8))))
    rows.append(m);print(json.dumps(m),flush=True)
    if seed==0:np.savez_compressed(OUT/(tag+'_'+law.replace('+','plus')+'_'+case+'.npz'),trace=z)
    (OUT/'metrics.json').write_text(json.dumps(rows,indent=2))
for case in ('friction_tracking','noise_tracking','slow_tracking'):
    for scale in (4.,16.):
        for seed in range(3):trial('gain_'+str(int(scale)),'C2',case,seed,scale=scale)
for law in ('C2','DQ-I','DQ-IM','C2+IM'):
    trial('long',law,'friction_tracking',T=40.)
for law in ('C2','DQ-I','DQ-IM'):
    trial('substeps10',law,'friction_tracking',substeps=10)
    trial('dt_half',law,'friction_tracking',dt=.0025)
    for seed in range(3):trial('smallpulse',law,'pulse_small',seed)
for factor in (.8,1.2):
    trial('frequency_'+str(factor),'DQ-IM','friction_tracking',harmonics=tuple(factor*np.array([.6,1.2,1.8])))
