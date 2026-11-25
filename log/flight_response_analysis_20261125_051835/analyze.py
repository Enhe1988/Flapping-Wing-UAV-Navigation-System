"""Read-only diagnosis of command timing and the final position-input failure."""
import csv,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'log/experiments/20261125_051835_flight_18621101607736'
OUT=Path(__file__).resolve().parent
def rows(name):
    with (P/name).open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def array(name,keys):return np.array([[float(x[k]) for k in keys] for x in rows(name)])
r=rows('reference.csv');t=np.array([float(x['t_s']) for x in r])
pos=array('control_position.csv',['t_s','x_m','y_m'])
cmd=array('velocity_sent.csv',['t_s','forward_wire_mps','lateral_wire_mps','valid'])
flow=array('velocity_actual.csv',['t_s','forward_mps','right_mps','velocity_valid'])
source=array('position_sources.csv',['t_s','raw_uwb_age_ms','reliable_uwb_age_ms','flow_age_ms','innovation_m'])
goals=np.array([[float(x['goal_x_m']),float(x['goal_y_m'])] for x in r])
map_velocity=np.array([[float(x['vx_map_desired_mps']),float(x['vy_map_desired_mps'])] for x in r])
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
axes[0].plot(t,goals[:,1],'--',label='Accepted desired map Y')
axes[0].plot(pos[:,0],pos[:,2],lw=.8,label='Used map Y')
axes[0].set_ylabel('Map Y [m]');axes[0].legend()
axes[1].plot(t,map_velocity[:,1],label='Computed map Y velocity')
axes[1].plot(cmd[:,0],cmd[:,2],label='UDP body right velocity',alpha=.8)
axes[1].set_ylabel('Command [m/s]');axes[1].legend()
axes[2].plot(flow[:,0],np.where(flow[:,3]==1,np.linalg.norm(flow[:,1:3],axis=1),np.nan),lw=.8,label='Reported XY speed magnitude')
axes[2].set_ylabel('Speed [m/s]');axes[2].set_xlabel('Time from heading alignment [s]');axes[2].legend()
for ax in axes:ax.axvline(26.405275155,color='red',ls=':',label='Goal accepted');ax.grid(alpha=.25);ax.set_xlim(25,42)
fig.suptitle('First waypoint: PC changes command immediately; actual following remains weak')
fig.tight_layout();fig.savefig(OUT/'waypoint_response.png',dpi=200);fig.savefig(OUT/'waypoint_response.pdf');plt.close(fig)
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
axes[0].plot(source[:,0],source[:,4],label='Raw UWB innovation');axes[0].axhline(.75,color='red',ls='--',label='Configured gate 0.75 m');axes[0].set_ylabel('Innovation [m]');axes[0].legend()
axes[1].plot(source[:,0],source[:,3],label='Flow sample age');axes[1].axhline(400,color='red',ls='--',label='Flow timeout 400 ms');axes[1].set_ylim(0,1600);axes[1].set_ylabel('Age [ms]');axes[1].legend()
axes[2].plot(cmd[:,0],np.linalg.norm(cmd[:,1:3],axis=1),label='UDP command magnitude');axes[2].step(cmd[:,0],cmd[:,3],where='post',label='Input valid flag',ls='--');axes[2].set_ylabel('m/s or flag');axes[2].set_xlabel('Time from heading alignment [s]');axes[2].legend()
for ax in axes:
    ax.axvline(113.465652684,color='tab:orange',ls=':');ax.axvline(115.465667978,color='red',ls='--');ax.set_xlim(110,118);ax.grid(alpha=.25)
fig.suptitle('Final zero command: UWB rejected + flow unavailable, then 2 s recovery expires')
fig.tight_layout();fig.savefig(OUT/'final_zero_cause.png',dpi=200);fig.savefig(OUT/'final_zero_cause.pdf');plt.close(fig)
summary={'session':str(P),'source_csv_modified':False,'goal_accepted_s':26.405275155,
         'first_changed_control_tick_s':26.425,'first_following_udp_send_s':26.429,
         'udp_send_max_interval_s':float(np.diff(cmd[:,0]).max()),
         'position_input_invalid_s':113.465652684,'goal_cleared_s':115.465667978,
         'last_accepted_filtered_uwb_s':float(rows('uwb_filtered.csv')[-1]['t_s']),
         'known':'PC emits changed commands immediately and later explicitly emits invalid zero frames because both position inputs are unusable.',
         'unknown':'FC receive latency, actual accepted velocity target, stick/mode arbitration, whether the final invalid observations coincided with landing/handling.'}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
