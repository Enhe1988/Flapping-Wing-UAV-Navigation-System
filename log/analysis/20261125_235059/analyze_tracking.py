from pathlib import Path
import csv,json,os
import numpy as np
root=next(p for p in Path(__file__).resolve().parents if (p/'dependencies.repos').is_file());session=root/'log/experiments/20261125_235059_flight_7429867014875';out=Path(__file__).parent

def rows(name):
    with (session/name).open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def t(r):return np.array([float(x['t_s']) for x in r])
ref=rows('reference.csv');position=rows('control_position.csv');pid=rows('attitude_pid.csv');raw=rows('trajectory.csv');filtered=rows('uwb_filtered.csv');sent=rows('attitude_sent.csv')
rt=t(ref);tracking=np.array([r['trajectory_phase']=='TRACKING' for r in ref]);lo,hi=rt[tracking][[0,-1]]
by_stamp={r['source_ros_ns']:r for r in position};used=np.array([[float(by_stamp[r['source_ros_ns']][k]) for k in ('x_m','y_m')] for r in ref]);goal=np.array([[float(r[k]) for k in ('goal_x_m','goal_y_m')] for r in ref]);velocity=np.array([[float(r[k]) for k in ('trajectory_vx_mps','trajectory_vy_mps')] for r in ref]);valid=tracking&np.array([r['inputs_valid']=='1' and by_stamp[r['source_ros_ns']]['valid']=='1' for r in ref]);err=goal-used
pt=t(pid);mask=(pt>=lo)&(pt<=hi)&np.array([r['valid']=='1' for r in pid]);I=np.array([[float(r[k]) for k in ('i_map_x_deg','i_map_y_deg')] for r in pid]);P=np.array([[float(r[k]) for k in ('p_map_x_deg','p_map_y_deg')] for r in pid]);D=np.array([[float(r[k]) for k in ('d_map_x_deg','d_map_y_deg')] for r in pid]);dt=np.r_[np.minimum(np.diff(pt),.1),0]
cap=mask&(np.linalg.norm(I,axis=1)>4.95);cap_times=pt[cap];summary={'session':session.name,'formal_tracking_start_old_t_s':float(lo),'formal_tracking_end_old_t_s':float(hi),'formal_tracking_duration_s':float(hi-lo),'automatic_during_tracking_confirmed_by_user':True,'manual_intervention':'only final shutdown/landing per user; exclude post-trajectory tail',
 'tracking_error_against_selected_uwb_not_truth':{'rmse_xy_m':np.sqrt(np.mean(err[valid]**2,axis=0)).tolist(),'distance_rmse_m':float(np.sqrt(np.mean(np.sum(err[valid]**2,axis=1)))),'p95_distance_m':float(np.percentile(np.linalg.norm(err[valid],axis=1),95)),'signed_mean_error_xy_m':np.mean(err[valid],axis=0).tolist()},
 'integral':{'near_5deg_limit_duration_s':float(sum(dt[cap])),'near_limit_first_last_old_t_s':[float(cap_times[0]),float(cap_times[-1])] if len(cap_times) else [],'max_abs_xy_deg':np.max(abs(I[mask]),axis=0).tolist(),'P_opposed_by_I_fraction_xy':[float(np.mean((P[mask,j]*I[mask,j]<0)&(abs(I[mask,j])>.5))) for j in range(2)]}}
# Descriptive time shift only; this includes closed-loop dynamics and is NOT
# UDP delay or an identified flight model. Search only within the measured run.
lags=[]
for j in range(2):
    scores=[]
    for lag in np.arange(0,6.001,.02):
        m=valid&(rt+lag<=hi)
        later=np.interp(rt[m]+lag,rt,used[:,j]);scores.append((float(np.sqrt(np.mean((later-goal[m,j])**2))),float(lag)))
    best=min(scores);lags.append({'best_shift_s':best[1],'rmse_after_shift_m':best[0]})
summary['descriptive_position_reference_time_shift_not_link_latency_xy']=lags
summary['gain_recommendation_not_applied']={'trial_1':{'attitude_integral_limit_deg':2.0,'other_PID':'keep P X10 Y13; I0.5 D3 unchanged'},'trial_2_if_X_still_lags_after_trial_1':{'attitude_kp_x_deg_per_m':12.0,'other_PID':'change one axis at a time; keep Y13 and D3'},'slower_reference_alternative':{'trajectory_figure8_period_s':100.0},'do_not_increase_blindly':['I','D','UWB filter time constant','dropout timeout']}
(out/'tracking_assessment.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
# A clean, explicitly cropped review plot; source CSV is never rewritten.
os.environ.setdefault('MPLCONFIGDIR','/tmp/flapping_tracking_review')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig=plt.figure(figsize=(11,6));grid=fig.add_gridspec(2,2);xy=fig.add_subplot(grid[:,0]);ax=fig.add_subplot(grid[0,1]);ay=fig.add_subplot(grid[1,1],sharex=ax)
for samples,label,color in [(raw,'Raw UWB','.75'),(filtered,'Online filtered UWB','tab:blue')]:
    ts=t(samples);m=(ts>=lo)&(ts<=hi);v=np.array([[float(r[k]) for k in ('x_m','y_m')] for r in samples]);v=v[m];times=ts[m]-lo
    xy.plot(v[:,0],v[:,1],'.',ms=1.2,label=label,color=color)
    plot_times=[];plot_values=[]
    for index,stamp in enumerate(times):
        if index and stamp-times[index-1]>.3:plot_times.append(np.nan);plot_values.append([np.nan,np.nan])
        plot_times.append(stamp);plot_values.append(v[index])
    plot_values=np.array(plot_values)
    for j,axis in enumerate((ax,ay)):axis.plot(plot_times,plot_values[:,j],color=color,lw=.8,label=label)
xy.plot(goal[tracking,0],goal[tracking,1],'r--',lw=1.2,label='Desired figure eight');xy.set_aspect('equal',adjustable='box');xy.set_xlabel('map X [m]');xy.set_ylabel('map Y [m]');xy.legend(fontsize=8,loc='lower left')
for j,axis in enumerate((ax,ay)):
    axis.plot(rt[tracking]-lo,goal[tracking,j],'r--',label='Desired');axis.set_ylabel('map '+('X' if j==0 else 'Y')+' [m]');axis.grid(alpha=.25)
ay.set_xlabel('Time since TRACKING started [s]');ax.legend(fontsize=8);fig.suptitle('Automatic figure eight: TRACKING only');fig.tight_layout(rect=(0,0,1,.95));fig.savefig(out/'tracking_only.png',dpi=220);fig.savefig(out/'tracking_only.pdf')
print(json.dumps(summary,ensure_ascii=False,indent=2))
