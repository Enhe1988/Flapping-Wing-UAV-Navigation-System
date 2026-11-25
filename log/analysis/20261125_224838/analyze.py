import csv,json,math
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'dependencies.repos').is_file())
OUT=Path(__file__).resolve().parent
NAMES=['20261125_224533_flight_3504200909832','20261125_224838_flight_3689049906810']

def read(root,name):
    with (root/name).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

def column(rows,name):return np.array([float(r[name]) for r in rows])

def describe(values):
    values=np.asarray(values);values=values[np.isfinite(values)]
    return {'min':float(np.min(values)),'median':float(np.median(values)),
            'p95':float(np.percentile(values,95)),'max':float(np.max(values))} if len(values) else None

def signed_margin(point,polygon):
    cross=[];dist=[]
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        d=b-a;cross.append(float(np.cross(d,point-a)))
        k=np.clip(np.dot(point-a,d)/np.dot(d,d),0,1);dist.append(np.linalg.norm(point-(a+k*d)))
    inside=min(cross)>=-1e-8 or max(cross)<=1e-8
    return float(min(dist))*(1 if inside else -1)

all_results=[]
for name in NAMES:
    root=ROOT/'log/experiments'/name
    session=json.loads((root/'session.json').read_text())
    reference=read(root,'reference.csv');positions=read(root,'control_position.csv')
    position_by_stamp={r['source_ros_ns']:r for r in positions}
    references=[r for r in reference if r['source_ros_ns'] in position_by_stamp]
    positions=[position_by_stamp[r['source_ros_ns']] for r in references]
    desired=read(root,'attitude_desired.csv');sent=read(root,'attitude_sent.csv')
    attitude=read(root,'attitude.csv');raw=read(root,'trajectory.csv');filtered=read(root,'uwb_filtered.csv')
    sources=read(root,'position_sources.csv')
    config=yaml.safe_load((root/'config/0_position_hold.yaml').read_text())['position_controller_node']['ros__parameters']
    scene=yaml.safe_load((root/'config/2_scene.yaml').read_text())['scene_visualizer_node']['ros__parameters']
    uwb=yaml.safe_load((root/'config/1_uwb.yaml').read_text())['uwb_position_node']['ros__parameters']
    anchors=np.array(scene['anchor_positions_m']).reshape(-1,3)[:,:2];anchors[:,1]*=uwb['uwb_y_sign']
    t=column(references,'t_s');p=np.array([[float(r['x_m']),float(r['y_m'])] for r in positions])
    goal=np.array([[float(r['goal_x_m']),float(r['goal_y_m'])] for r in references])
    error=goal-p;distance=np.linalg.norm(error,axis=1)
    changes=np.r_[0,np.where(np.any(np.diff(goal,axis=0)!=0,axis=1))[0]+1]
    boundaries=np.r_[changes,len(t)]
    spans=[]
    for k,(i,j) in enumerate(zip(boundaries[:-1],boundaries[1:])):
        points=p[i:j];g=goal[i];e=distance[i:j]
        entry=np.where(e<=.1)[0]
        item={'goal_number':k,'start_s':float(t[i]),'end_s':float(t[j] if j<len(t) else t[-1]),
              'goal_x_m':float(g[0]),'goal_y_m':float(g[1]),
              'initial_error_m':float(e[0]),'min_error_m':float(np.min(e)),
              'final_error_m':float(e[-1]),'rmse_m':float(np.sqrt(np.mean(e*e))),
              'fraction_within_0_1m':float(np.mean(e<=.1)),
              'first_within_0_1m_s':float(t[i+entry[0]]-t[i]) if len(entry) else None,
              'anchor_polygon_margin_m':signed_margin(g,anchors)}
        if k:
            delta=g-goal[changes[k-1]]
            for ax,label in enumerate(['x','y']):
                item[label+'_overshoot_m']=float(max(0,np.max(np.sign(delta[ax])*(points[:,ax]-g[ax])))) if abs(delta[ax])>.1 else None
        spans.append(item)
    angle=np.array([[float(r['pitch_deg']),float(r['roll_deg'])] for r in desired])
    st=column(sent,'t_s');sa=np.array([[float(r['pitch_deg']),float(r['roll_deg'])] for r in sent])
    dt=column(desired,'t_s')
    ds=np.array([int(r['source_ros_ns']) for r in desired],dtype=np.int64)
    ss=np.array([int(r['source_ros_ns']) for r in sent],dtype=np.int64)
    ix=np.clip(np.searchsorted(ds,ss,side='right')-1,0,len(ds)-1)
    quant=sa-angle[ix]
    rawp=np.array([[float(r['x_m']),float(r['y_m'])] for r in raw])
    filt_by_stamp={r['source_ros_ns']:r for r in filtered}
    matched=[r for r in raw if r['source_ros_ns'] in filt_by_stamp]
    shifts=[np.hypot(float(r['x_m'])-float(filt_by_stamp[r['source_ros_ns']]['x_m']),
                     float(r['y_m'])-float(filt_by_stamp[r['source_ros_ns']]['y_m'])) for r in matched]
    result={'session':name,'duration_s':float(t[-1]),'flight_mode':session['flight_mode'],
            'user_reported_manual_intervention':True if name==NAMES[-1] else 'unknown',
            'kp_x_deg_per_m':config['attitude_kp_x_deg_per_m'],'kp_y_deg_per_m':config['attitude_kp_y_deg_per_m'],
            'position_tolerance_m':config['arrival_position_tolerance_m'],
            'navigation_invalid_count':sum(r['inputs_valid']!='1' for r in references),
            'goal_cleared_count':sum(r['goal_active']!='1' for r in references),
            'sent_invalid_count':sum(r['valid']!='1' for r in sent),
            'sent_interval_ms':describe(np.diff(st)*1000),
            'sent_command_age_ms':describe(column(sent,'command_age_ms')),
            'sent_yaw_age_ms':describe(column(sent,'attitude_age_ms')),
            'max_abs_pitch_deg':float(np.max(abs(angle[:,0]))),'max_abs_roll_deg':float(np.max(abs(angle[:,1]))),
            'pitch_saturation_fraction':float(np.mean(abs(angle[:,0])>=29.99)),
            'roll_saturation_fraction':float(np.mean(abs(angle[:,1])>=29.99)),
            'pitch_slew_p99_deg_s':float(np.percentile(abs(np.diff(angle[:,0])/np.diff(dt)),99)),
            'roll_slew_p99_deg_s':float(np.percentile(abs(np.diff(angle[:,1])/np.diff(dt)),99)),
            'wire_minus_latest_published_deg':describe(abs(quant).reshape(-1)),
            'raw_uwb_interval_ms':describe(np.diff(column(raw,'t_s'))*1000),
            'raw_rows':len(raw),'filtered_rows':len(filtered),'raw_to_filtered_offset_m':describe(shifts),
            'yaw_interval_ms':describe(np.diff(column(attitude,'t_s'))*1000),
            'yaw_map_deg':describe(column(attitude,'yaw_map_deg')),
            'states':{s:sum(r['state']==s for r in references) for s in sorted({r['state'] for r in references})},
            'goal_segments':spans,'recorder_dropped':session['dropped_messages']}
    desired_by_stamp={r['source_ros_ns']:r for r in desired}
    result['goal_command_response']=[]
    for i in changes[1:]:
        stamp=references[i]['source_ros_ns'];before=desired_by_stamp[references[i-1]['source_ros_ns']]
        current=desired_by_stamp[stamp];first_wire=next(r for r in sent if int(r['source_ros_ns'])>=int(stamp))
        result['goal_command_response'].append({'goal_time_s':float(t[i]),
            'same_control_tick_pitch_change_deg':float(current['pitch_deg'])-float(before['pitch_deg']),
            'same_control_tick_roll_change_deg':float(current['roll_deg'])-float(before['roll_deg']),
            'first_following_udp_monitor_ms':(int(first_wire['source_ros_ns'])-int(stamp))/1e6})
    result['initial_15s_near_static_xy_std_m']={}
    for label,rows in [('raw',raw),('filtered',filtered)]:
        sample=np.array([[float(r['x_m']),float(r['y_m'])] for r in rows if 0<float(r['t_s'])<15])
        result['initial_15s_near_static_xy_std_m'][label]=np.std(sample,axis=0).tolist()
    all_results.append(result)
    if name!=NAMES[-1]:continue
    grid=np.arange(t[0],t[-1],.02)
    offline_position=np.array([np.interp(grid,t,p[:,ax]) for ax in range(2)]).T
    # Analysis only: symmetric ~0.42s Savitzky-Golay window on recorded UWB.
    # This is neither logged optical-flow feedback nor an onboard measurement.
    smooth=savgol_filter(offline_position,21,2,axis=0)
    estimated_velocity=savgol_filter(offline_position,21,2,deriv=1,delta=.02,axis=0)
    with (OUT/'uwb_velocity_estimate_ANALYSIS_ONLY.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['t_s','x_smoothed_m','y_smoothed_m','vx_uwb_estimated_mps','vy_uwb_estimated_mps'])
        w.writerows(np.column_stack([grid,smooth,estimated_velocity]))
    for a,b in [(75.44,82),(36.34,44.46),(44.46,67.42)]:
        mask=(grid>=a)&(grid<=b);result.setdefault('offline_uwb_derived_speed_windows',[]).append({
            'start_s':a,'end_s':b,'max_abs_vx_mps':float(max(abs(estimated_velocity[mask,0]))),
            'max_abs_vy_mps':float(max(abs(estimated_velocity[mask,1])))})
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.25})
    fig,axes=plt.subplots(5,1,figsize=(12,12),sharex=True)
    for axindex,label in enumerate(['X','Y']):
        axes[axindex].plot(t,p[:,axindex],label='Filtered UWB position')
        axes[axindex].step(t,goal[:,axindex],where='post',linestyle='--',label='Requested waypoint')
        axes[axindex].set_ylabel(label+' [m]');axes[axindex].legend(loc='upper right')
    axes[2].plot(dt,angle[:,0],'--',label='Desired pitch');axes[2].plot(st,sa[:,0],label='UDP pitch')
    axes[2].plot(dt,angle[:,1],'--',label='Desired roll');axes[2].plot(st,sa[:,1],label='UDP roll')
    axes[2].axhline(30,color='grey',linestyle=':');axes[2].axhline(-30,color='grey',linestyle=':')
    axes[2].set_ylabel('Target [deg]');axes[2].legend(ncol=4)
    axes[3].plot(column(attitude,'t_s'),column(attitude,'yaw_map_deg'),label='Measured yaw relative to alignment')
    axes[3].set_ylabel('Yaw [deg]');axes[3].legend()
    axes[4].plot(grid,estimated_velocity[:,0],label='X speed estimated offline from UWB')
    axes[4].plot(grid,estimated_velocity[:,1],label='Y speed estimated offline from UWB')
    axes[4].set_ylabel('Estimate [m/s]');axes[4].set_xlabel('Seconds since heading alignment');axes[4].legend()
    for ax in axes:
        for i in changes[1:]:ax.axvline(t[i],color='grey',alpha=.4,linewidth=.8)
    fig.suptitle('2026-11-25 22:48:38 flight: waypoint control and sent angles\nManual intervention reported; timing is not labeled; no measured roll/pitch telemetry')
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(OUT/'assessment.png',dpi=200);fig.savefig(OUT/'assessment.pdf');plt.close(fig)
    # Detail the large final X waypoint step; do not label this as a pure auto response.
    fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
    axes[0].plot(t,p[:,0],label='Actual UWB X');axes[0].step(t,goal[:,0],where='post',linestyle='--',label='Waypoint X')
    axes[0].set_ylabel('X [m]');axes[0].legend()
    axes[1].plot(grid,estimated_velocity[:,0],label='Offline UWB-derived X speed');axes[1].axhline(0,color='grey');axes[1].set_ylabel('Estimate [m/s]');axes[1].legend()
    axes[2].plot(st,sa[:,0],label='Sent pitch');axes[2].plot(st,sa[:,1],label='Sent roll');axes[2].set_ylabel('Angle [deg]');axes[2].legend()
    for ax in axes:ax.axvline(t[changes[-1]],color='grey',linestyle=':');ax.set_xlim(74,83)
    axes[-1].set_xlabel('Seconds since alignment');fig.suptitle('Last waypoint change: observed motion (manual intervention timing unknown)')
    fig.tight_layout();fig.savefig(OUT/'last_waypoint.png',dpi=200);fig.savefig(OUT/'last_waypoint.pdf');plt.close(fig)

summary={'source_csv_modified':False,'sessions':all_results,
         'limitations':['Manual intervention has been confirmed; affected times are unknown.',
                        'UDP sent is a local monitor, not FC receipt or attitude-loop execution acknowledgment.',
                        'Measured roll/pitch and measured optical-flow speed are not available.',
                        'UWB-derived offline speed uses resampling and symmetric smoothing; not real-time feedback.']}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
with (OUT/'goal_segments.csv').open('w',newline='') as f:
    fields=sorted({k for row in all_results[-1]['goal_segments'] for k in row});w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(all_results[-1]['goal_segments'])
print(json.dumps(summary,indent=2))
