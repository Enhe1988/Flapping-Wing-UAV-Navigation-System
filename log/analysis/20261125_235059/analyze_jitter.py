import csv,json
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfiltfilt
root=next(p for p in Path(__file__).resolve().parents if (p/'dependencies.repos').is_file());session=root/'log/experiments/20261125_235059_flight_7429867014875';out=Path(__file__).parent
start=json.loads((session/'session.json').read_text())['start_ros_ns']
def load(name):
    with (session/name).open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def t(rows):return np.array([(int(r['source_ros_ns'])-start)/1e9 for r in rows])
def xy(rows):return np.array([[float(r['x_m']),float(r['y_m'])] for r in rows])
raw=load('trajectory.csv');filtered=load('uwb_filtered.csv');nav=load('reference.csv');pid=load('attitude_pid.csv');sent=load('attitude_sent.csv');att=load('attitude.csv')
rt,ft=t(raw),t(filtered);rp,fp=xy(raw),xy(filtered)
ref_t=t(nav);g=np.array([[float(r['goal_x_m']),float(r['goal_y_m'])] for r in nav]);tracking=np.array([r['trajectory_phase']=='TRACKING' for r in nav]);a,b=ref_t[tracking][[0,-1]]
summary={'session':session.name,'duration_s':float(rt[-1]),'tracking_s':[float(a),float(b)],'input_invalid_rows':sum(r['inputs_valid']!='1' for r in nav),'goal_missing_rows':sum(r['goal_active']!='1' for r in nav),'raw_samples':len(raw),'filtered_samples':len(filtered),'position_statistics':{}}
def stats(stamp,p):
    mask=(stamp>=a)&(stamp<=b);times=stamp[mask];pos=p[mask];delta=np.linalg.norm(np.diff(pos,axis=0),axis=1);dt=np.diff(times)
    grid=np.arange(times[0],times[-1],.02);uniform=np.column_stack([np.interp(grid,times,pos[:,j]) for j in range(2)])
    hf=sosfiltfilt(butter(3,1,fs=50,btype='highpass',output='sos'),uniform,axis=0)
    return {'rate_hz':float(1/np.median(dt)),'max_gap_ms':float(max(dt)*1000),'step_cm_percentiles':dict(zip(['p50','p95','p99','max'],map(float,np.percentile(delta,[50,95,99,100])*100))),'above_1hz_component_rms_cm_xy':(np.sqrt(np.mean(hf**2,axis=0))*100).tolist()}
summary['position_statistics']['raw']=stats(rt,rp);summary['position_statistics']['filtered_tau_010']=stats(ft,fp)
rawdict={r['source_ros_ns']:np.array([float(r['x_m']),float(r['y_m'])]) for r in raw}
delta=np.array([rawdict[r['source_ros_ns']]-np.array([float(r['x_m']),float(r['y_m'])]) for r in filtered if r['source_ros_ns'] in rawdict])
summary['raw_minus_filtered_cm_percentiles']=dict(zip(['p50','p95','p99','max'],map(float,np.percentile(np.linalg.norm(delta,axis=1),[50,95,99,100])*100)))
used=load('control_position.csv');ut=t(used);up=xy(used);upos=np.column_stack([np.interp(ref_t,ut,up[:,j]) for j in range(2)]);err=upos[tracking]-g[tracking]
summary['tracking_error_against_selected_uwb_not_truth']={'rmse_xy_m':np.sqrt(np.mean(err**2,axis=0)).tolist(),'distance_rmse_m':float(np.sqrt(np.mean(np.sum(err**2,axis=1)))),'distance_p95_m':float(np.percentile(np.linalg.norm(err,axis=1),95))}
summary['command_stats']={'sent_invalid_rows':sum(r['valid']!='1' for r in sent),'max_abs_pitch_roll_deg':[max(abs(float(r[k])) for r in sent) for k in ('pitch_deg','roll_deg')],'max_abs_I_xy_deg':[max(abs(float(r[k])) for r in pid) for k in ('i_map_x_deg','i_map_y_deg')],'max_abs_D_xy_deg':[max(abs(float(r[k])) for r in pid) for k in ('d_map_x_deg','d_map_y_deg')]}
yaw=np.unwrap(np.array([float(r['yaw_map_deg']) for r in att])*np.pi/180)*180/np.pi
summary['yaw']={'valid_rows':sum(r['attitude_valid']=='1' for r in att),'sample_count':len(att),'largest_adjacent_change_deg':float(max(abs(np.diff(yaw)))),'adjacent_change_p95_deg':float(np.percentile(abs(np.diff(yaw)),95))}
# Offline replay of the SAME recorded observations, no flight or controller run.
def lowpass(t,p,tau):
    result=p.copy()
    for i in range(1,len(t)):
        h=t[i]-t[i-1]
        if h>.3:continue
        alpha=-np.expm1(-h/tau);result[i]=result[i-1]+alpha*(p[i]-result[i-1])
    return result
alternatives={tau:lowpass(rt,rp,tau) for tau in (.10,.15,.20,.30)}
summary['offline_only_filter_comparison']={str(tau):stats(rt,p) for tau,p in alternatives.items()}
summary['caveat']='No independent ground truth, actual roll/pitch or manual-takeover telemetry. Above-1Hz components may contain real flight motion; they are not proven measurement error. Offline filter replay cannot establish closed-loop performance.'
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');print(json.dumps(summary,ensure_ascii=False,indent=2))
# Show a representative moving interval plus the initial nearly stationary preparation.
import os;os.environ.setdefault('MPLCONFIGDIR','/tmp/flapping_jitter_matplotlib')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,2,figsize=(12,6),sharex='col')
for j,axis in enumerate('XY'):
    for col,(lo,hi,title) in enumerate([(0,12,'Preparation interval'),(72,82,'Figure-eight interval')]):
        m=(rt>=lo)&(rt<=hi)
        axes[j,col].plot(rt[m],rp[m,j],color='.65',lw=.8,label='Raw UWB (current aircraft display)')
        for tau,p in alternatives.items():
            axes[j,col].plot(rt[m],p[m,j],lw=1,label=f'Offline LPF tau={tau:.2f}s')
        axes[j,col].set_ylabel(f'map {axis} [m]');axes[j,col].grid(alpha=.25)
        if j==0:axes[j,col].set_title(title)
        if j==1:axes[j,col].set_xlabel('Time since alignment [s]')
axes[0,0].legend(fontsize=7);fig.suptitle('Recorded UWB: raw display, existing filter and offline alternatives (not ground truth)');fig.tight_layout();fig.savefig(out/'uwb_jitter_comparison.png',dpi=180);fig.savefig(out/'uwb_jitter_comparison.pdf')
