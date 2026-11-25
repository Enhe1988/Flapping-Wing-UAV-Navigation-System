"""Offline diagnosis and planned-path comparison; never changes recorded flight CSV."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
SESSION=ROOT/'log/experiments/20261125_040156_flight_14021628869426'
def rows(name):
    with (SESSION/name).open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def array(name,columns):return np.array([[float(r[k]) for k in columns] for r in rows(name)])
r=rows('reference.csv');t=np.array([float(x['t_s']) for x in r])
pos=array('control_position.csv',['t_s','x_m','y_m'])
used=np.column_stack([np.interp(t,pos[:,0],pos[:,k]) for k in (1,2)])
goals=np.array([[float(x['goal_x_m']),float(x['goal_y_m'])] for x in r])
error=np.linalg.norm(goals-used,axis=1)
sent=array('velocity_sent.csv',['t_s','forward_wire_mps','lateral_wire_mps','valid'])
actual=array('velocity_actual.csv',['t_s','forward_mps','right_mps','velocity_valid'])
actual[actual[:,3]!=1,1:3]=np.nan
ff=np.array([[float(x['trajectory_vx_mps']),float(x['trajectory_vy_mps'])] for x in r])
summary={'session':str(SESSION),'source_data_modified':False,'phases':{}}
for phase in ('WAITING_START','APPROACH','TRACKING','FINISHED'):
    mask=np.array([x['trajectory_phase']==phase for x in r]);lo,hi=t[mask][[0,-1]]
    c=sent[(sent[:,0]>=lo)&(sent[:,0]<=hi)]
    a=actual[(actual[:,0]>=lo)&(actual[:,0]<=hi)]
    cs=np.linalg.norm(c[:,1:3],axis=1);vs=np.linalg.norm(a[:,1:3],axis=1)
    summary['phases'][phase]={'start_s':float(lo),'end_s':float(hi),'duration_s':float(hi-lo),
        'median_position_error_m':float(np.median(error[mask])),
        'maximum_position_error_m':float(np.max(error[mask])),
        'median_udp_command_speed_mps':float(np.median(cs)),
        'command_near_2mps_fraction':float(np.mean(cs>=1.99)),
        'median_reported_speed_mps':float(np.nanmedian(vs)),
        'peak_reference_speed_mps':float(np.max(np.linalg.norm(ff[mask],axis=1)))}

fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
axes[0].plot(t,error,lw=.9);axes[0].set_ylabel('Position error [m]')
axes[1].plot(sent[:,0],np.linalg.norm(sent[:,1:3],axis=1),label='UDP command',lw=.9)
axes[1].plot(actual[:,0],np.linalg.norm(actual[:,1:3],axis=1),label='Reported flow speed',lw=.8)
axes[1].plot(t,np.linalg.norm(ff,axis=1),label='Reference speed',lw=1.2)
axes[1].set_ylabel('XY speed [m/s]');axes[1].legend(loc='upper left',ncol=3)
axes[2].plot(t,goals[:,0],label='Desired X',color='tab:blue',ls='--')
axes[2].plot(pos[:,0],pos[:,1],label='Used X',color='tab:blue',lw=.8)
axes[2].plot(t,goals[:,1],label='Desired Y',color='tab:orange',ls='--')
axes[2].plot(pos[:,0],pos[:,2],label='Used Y',color='tab:orange',lw=.8)
axes[2].set_ylabel('Map position [m]');axes[2].set_xlabel('Time from heading alignment [s]');axes[2].legend(ncol=4)
for ax in axes:
    for phase,color in [('APPROACH','tab:green'),('TRACKING','tab:red')]:
        s=summary['phases'][phase];ax.axvspan(s['start_s'],s['end_s'],color=color,alpha=.06)
    ax.grid(alpha=.25)
axes[0].set_title('Latest recorded flight: 2026-11-25 04:01:56 (raw logged values)')
fig.tight_layout();fig.savefig(OUT/'latest_response.png',dpi=200);fig.savefig(OUT/'latest_response.pdf');plt.close(fig)

share=ROOT/'src/flapping_uav_bringup'
spec=importlib.util.spec_from_file_location('geometry',share/'launch/trajectory_geometry.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
c=yaml.safe_load((share/'config/trajectory.yaml').read_text())['trajectory']
scene=yaml.safe_load((share/'config/scene.yaml').read_text())['scene_visualizer_node']['ros__parameters']
uwb=yaml.safe_load((share/'config/uwb.yaml').read_text())['uwb_position_node']['ros__parameters']
new=g.geometry(scene,uwb,c['boundary_margin_m'],c['figure8_fit_scale'],c['line_length_m'],c['figure8_axis'],c['figure8_lobe_aspect_ratio'])
old=json.loads((SESSION/'parameters/position_controller_node.json').read_text())
old_extents=json.loads(old['trajectory_half_extents_xy']['value_text'])
old_center=json.loads(old['trajectory_center_xy']['value_text'])
phase=np.linspace(0,2*math.pi,2001)
poly=np.array(new['polygon']).reshape(-1,2);closed=np.vstack((poly,poly[0]))
fig,axes=plt.subplots(1,2,figsize=(10,9),sharex=True,sharey=True)
for ax,center,extents,along_y,title in [
    (axes[0],old_center,old_extents,False,'Previous plan: X lobes, 40 s period'),
    (axes[1],new['center'],new['figure8_half_extents'],True,'New plan: Y lobes, 80 s period')]:
    x=center[0]+extents[0]*np.sin((2 if along_y else 1)*phase)
    y=center[1]+extents[1]*np.sin((1 if along_y else 2)*phase)
    ax.fill(closed[:,0],closed[:,1],color='0.96');ax.plot(closed[:,0],closed[:,1],color='0.4')
    ax.plot(x,y,lw=2,color='tab:blue');ax.plot(*center,'o',color='tab:red',label='Start / center')
    for idx,(px,py) in enumerate(poly):ax.annotate('({:.2f}, {:.2f})'.format(px,py),(px,py),xytext=(3,5),textcoords='offset points',fontsize=8)
    ax.set_title(title);ax.set_xlabel('Map X [m]');ax.set_aspect('equal');ax.grid(alpha=.25)
    ax.set_xlim(-.5,4.8);ax.set_ylim(-8.2,.5);ax.legend(loc='upper right')
axes[0].set_ylabel('Map Y [m]')
fig.suptitle('Horizontal XY paths within the current anchor area; new plan is not flight evidence')
fig.tight_layout();fig.savefig(OUT/'figure8_plan_comparison.png',dpi=200);fig.savefig(OUT/'figure8_plan_comparison.pdf');plt.close(fig)
summary['new_plan']=dict(new,configured_period_s=c['figure8_period_s'],
    conservative_peak_speed_mps=math.hypot(2*new['figure8_half_extents'][0],new['figure8_half_extents'][1])*2*math.pi/c['figure8_period_s'])
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
