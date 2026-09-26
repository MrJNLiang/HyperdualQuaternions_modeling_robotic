"""Publication-style static plots; all values loaded from completed runs."""
from pathlib import Path
import json
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
PLOTS=HERE/'figures';PLOTS.mkdir(exist_ok=True)
rows=json.loads((HERE/'results/metrics.json').read_text())+json.loads((HERE/'results_im/metrics.json').read_text())
sens=json.loads((HERE/'sensitivity_results/metrics.json').read_text())
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':140,'savefig.dpi':180})
colors={'C2':'#59677c','DQ-I':'#227c9d','DQ-IM':'#dd6838','C2+IM':'#6b4ca0'}

def mean(case,law,key,rr=rows,tag=None):
    g=[r for r in rr if r['case']==case and r['law']==law and (tag is None or r.get('tag')==tag)]
    assert g and not any(r['failed'] for r in g)
    return float(np.mean([r[key] for r in g]))

def trace(case,law):
    root='results_im' if law in ('DQ-IM','C2+IM') else 'results'
    return np.load(HERE/root/(case+'_'+law.replace('+','plus')+'.npz'))['trace']

fig,axs=plt.subplots(2,2,figsize=(11,6.4),sharex=True)
for col,(case,title) in enumerate([('slow_tracking','Slowly varying external torque'),('friction_tracking','Friction + model mismatch')]):
    for law in ('C2','DQ-I','DQ-IM'):
        z=trace(case,law)
        axs[0,col].plot(z[:,0],1000*z[:,1],label=law,color=colors[law],lw=1.6)
        axs[1,col].plot(z[:,0],z[:,2]*180/np.pi,color=colors[law],lw=1.6)
    axs[0,col].set_title(title);axs[1,col].set_xlabel('Time (s)')
    for ax in axs[:,col]:ax.grid(alpha=.2);ax.axvspan(11.2,16,color='#c7d8cc',alpha=.15)
axs[0,0].set_ylabel('TCP position error (mm)');axs[1,0].set_ylabel('Orientation error (deg)')
axs[0,0].legend(ncol=3,loc='upper right',fontsize=9)
fig.suptitle('B601 offline torque simulation | seed 0 | shaded: evaluation window',fontsize=12)
fig.tight_layout();fig.savefig(PLOTS/'tracking.png');plt.close(fig)

labels=['C2','C2 gain ×4','C2 gain ×16','DQ-I','DQ-IM']
configs=[('C2',rows,None),('C2',sens,'gain_4'),('C2',sens,'gain_16'),('DQ-I',rows,None),('DQ-IM',rows,None)]
barcolors=[colors['C2'],'#8798b2','#bac5d5',colors['DQ-I'],colors['DQ-IM']]
fig,axs=plt.subplots(1,3,figsize=(12.8,4.6))
for ax,case,key,title,unit in zip(axs,
 ['friction_tracking','friction_tracking','noise_tracking'],
 ['pos_rms_mm','angle_rms_deg','torque_rate_rms'],
 ['Friction: position RMS','Friction: orientation RMS','Noise: torque-rate RMS'],
 ['mm','deg','N m / s']):
    vals=[mean(case,law,key,rr,tag) for law,rr,tag in configs]
    ax.bar(range(5),vals,color=barcolors,width=.66)
    for i,v in enumerate(vals):ax.text(i,v+max(vals)*.025,format(v,'.3g'),ha='center',fontsize=9)
    ax.set_xticks(range(5));ax.set_xticklabels(labels,rotation=27,ha='right');ax.set_title(title);ax.set_ylabel(unit)
    ax.set_ylim(0,max(vals)*1.2);ax.grid(axis='y',alpha=.2)
fig.suptitle('Accuracy and effort tradeoff | means over 3 seeds | C2 gain: Kp multiplier, Kd sqrt multiplier',fontsize=11)
fig.tight_layout();fig.savefig(PLOTS/'tradeoff.png');plt.close(fig)

f=np.load(HERE/'results/frequency.npz');g=np.load(HERE/'results/frequency_im.npz')
fig,axs=plt.subplots(1,2,figsize=(11,4.4))
for law,omega,mag in [('C2',f['omega'],f['C2']),('DQ-I',f['omega'],f['candidate']),('DQ-IM',g['omega'],g['candidate'])]:
    axs[0].loglog(omega,mag,color=colors[law],label=law,lw=1.7)
axs[0].axhline(1/36,color='black',ls=':',lw=1,label='Guaranteed bound 1/36')
axs[0].set(xlim=(.03,40),ylim=(1e-5,.04),xlabel='Disturbance frequency (rad/s)',ylabel='Pose / acceleration gain (s²)',title='Local frequency response')
axs[0].legend(fontsize=8);axs[0].grid(which='both',alpha=.15)
vr=json.loads((HERE/'results/verification_im.json').read_text())
for row in vr['nonlinear']:
    if row['modeled_disturbance']:continue
    gamma=row['gamma'];a=np.load(HERE/'results'/('im_certificate_'+str(round(gamma,5))+'_0.npz'))
    axs[1].plot(a['t'],a['output_energy']/(gamma**2*a['input_energy'][-1]),label='gamma = '+format(gamma,'.4g'))
axs[1].axhline(1,color='black',ls=':',lw=1)
axs[1].set(xlim=(0,120),ylim=(0,1.03),xlabel='Time (s)',ylabel='Output energy / allowed total energy',title='Nonlinear certificate, zero initial error')
axs[1].legend(fontsize=9);axs[1].grid(alpha=.2)
fig.tight_layout();fig.savefig(PLOTS/'certificate.png');plt.close(fig)

# Machine-readable aggregate: never mix truncated failure trajectories into RMS comparisons.
summary={}
for case in sorted(set(r['case'] for r in rows)):
    summary[case]={}
    for law in sorted(set(r['law'] for r in rows)):
        group=[r for r in rows if r['case']==case and r['law']==law]
        if not group:continue
        item={'runs':len(group),'failures':sum(r['failed'] for r in group)}
        if not item['failures']:
            for key in ('pos_rms_mm','angle_rms_deg','pos_all_mm','angle_all_deg','torque_rms','torque_rate_rms','jerk_rms','limited_fraction'):
                item[key]={'mean':float(np.mean([r[key] for r in group])),
                           'min':min(r[key] for r in group),'max':max(r[key] for r in group)}
        summary[case][law]=item
(HERE/'summary.json').write_text(json.dumps(summary,indent=2))
print('Saved 3 figures and summary.json')
