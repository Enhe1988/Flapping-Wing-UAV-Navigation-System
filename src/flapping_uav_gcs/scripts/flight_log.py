#!/usr/bin/env python3
"""Experiment control and offline Foxy rosbag SQLite -> CSV/PNG/PDF/HTML export.

Recorders are C++/rclcpp. This tool never sends flight commands; its online
commands address only /experiment/*. Offline export does not initialize DDS.
"""
import argparse
import base64
import csv
import html
import json
import math
import numbers
import os
from pathlib import Path
import re
import sqlite3
import subprocess
import sys
import time


def load_json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {} if default is None else default


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def finite(value):
    return isinstance(value, numbers.Real) and math.isfinite(value)


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def config_values(session):
    import yaml
    data = {}
    for path in sorted((session / 'config').glob('*.yaml')):
        with path.open(encoding='utf-8') as stream:
            value = yaml.safe_load(stream)
        if isinstance(value, dict):
            data.update(value)
    params = {k: dict(v.get('ros__parameters', {})) for k, v in data.items() if isinstance(v, dict) and 'ros__parameters' in v}
    # Actual running node parameters take priority over files edited before restart.
    for path in (session / 'parameters').glob('*.json'):
        node = path.stem
        target = params.setdefault(node, {})
        for key, item in load_json(path).items():
            try:
                value = yaml.safe_load(item['value_text'])
                target[key] = value
            except (KeyError, ValueError, yaml.YAMLError):
                pass
    return data, params


def flatten_fields(value, prefix=''):
    """Flatten ROS fields for lossless, per-topic tabular inspection."""
    if hasattr(value, 'get_fields_and_field_types'):
        out = {}
        for field in value.get_fields_and_field_types():
            out.update(flatten_fields(getattr(value, field), prefix + field + '.'))
        return out
    key = prefix[:-1]
    if hasattr(value, 'item') and not hasattr(value, '__len__'):
        value = value.item()
    if isinstance(value, (str, bool, int, float)) or value is None:
        return {key: value}
    try:
        array = list(value)
        # Fixed channels/axes get distinct CSV columns; nested variable arrays
        # remain JSON so ranges/diagnostic frames are not silently discarded.
        if all(isinstance(x, numbers.Real) for x in array) and len(array) <= 32:
            return {key + '[' + str(i) + ']': x for i, x in enumerate(array)}
        return {key: json.dumps([message_value(x) for x in array], ensure_ascii=False)}
    except TypeError:
        return {key: str(value)}


def message_value(value):
    if hasattr(value, 'item') and not hasattr(value, '__len__'):
        value = value.item()
    if hasattr(value, 'get_fields_and_field_types'):
        return {k: message_value(getattr(value, k)) for k in value.get_fields_and_field_types()}
    if isinstance(value, (str, bool, int, float)) or value is None:
        return value
    return [message_value(x) for x in value]


def euler(orientation, inverse=False):
    q = [orientation.w, orientation.x, orientation.y, orientation.z]
    if not all(finite(x) for x in q):
        return [math.nan] * 3
    norm = math.sqrt(sum(x*x for x in q))
    if abs(norm-1.) > .05 or norm == 0:
        return [math.nan] * 3
    w, x, y, z = [v/norm for v in q]
    if inverse:
        x, y, z = -x, -y, -z
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def records(session, manifest, errors):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    timeline = {}
    path = session / 'timeline.csv'
    if path.exists():
        with path.open(newline='', encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                timeline[int(row['message_id'])] = row
    classes = {}
    for db in sorted((session / 'bag').glob('*.db3')):
        # mode=ro forbids accidentally changing the recorded bag.
        connection = sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True)
        topics = {row[0]: (row[1], row[2]) for row in connection.execute('SELECT id,name,type FROM topics')}
        try:
            for message_id, topic_id, timestamp, blob in connection.execute('SELECT id,topic_id,timestamp,data FROM messages ORDER BY id'):
                topic, typename = topics[topic_id]
                timing = timeline.get(message_id)
                if timing and timing['topic'] == topic and int(timing['receive_ros_ns']) == timestamp:
                    monotonic = int(timing['receive_steady_ns'])
                    t = (monotonic-manifest['start_steady_ns'])/1e9
                else:
                    monotonic = None
                    t = (timestamp-manifest['start_ros_ns'])/1e9
                    if 'missing_monotonic_timeline' not in errors:
                        errors.append('missing_monotonic_timeline')
                try:
                    if typename not in classes:
                        classes[typename] = get_message(typename)
                    yield topic, timestamp, monotonic, t, deserialize_message(blob, classes[typename])
                except Exception as exc:
                    errors.append('deserialize {} #{}: {}'.format(topic, message_id, exc))
        finally:
            connection.close()


def write_csv(path, rows):
    if not rows:
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_session(session, recover=False):
    manifest = load_json(session/'session.json')
    if not manifest or 'start_steady_ns' not in manifest:
        raise ValueError('Not an experiment directory: ' + str(session))
    if manifest['state'] == 'recording' and not recover:
        raise ValueError('Session is still recording. Stop it before export; raw data is untouched.')
    interrupted = manifest['state'] == 'recording'
    if interrupted:
        manifest = dict(manifest)
        manifest['recording_error'] = 'Interrupted/unfinalized recording; explicitly recovered from committed SQLite data'
    data, params = config_values(session)
    control = params.get('position_controller_node', {})
    contract = data.get('frame_contract', {})
    timeout_s = float(control.get('position_timeout_ms', 500))/1000
    rows, specialized, events, errors, counts = {}, {}, [], [], {}
    last_pose = None
    alignment = None
    last_nav = None
    last_fusion = None
    goal_id = 0
    source_times = {}
    previous_diag = None
    inverse = not bool(control.get('quaternion_body_to_ned', contract.get('quaternion_body_to_ned', True)))
    for topic, timestamp, monotonic, t, m in records(session, manifest, errors):
        counts[topic] = counts.get(topic, 0)+1
        row = {'t_s': t, 'receive_ros_ns': timestamp, 'receive_steady_ns': monotonic}
        row.update(flatten_fields(m))
        rows.setdefault(topic, []).append(row)
        if topic == '/uwb/position':
            p = m.pose.pose.position
            last_pose = (t, p.x, p.y, p.z)
            specialized.setdefault('position', []).append({'t_s': t, 'x_m': p.x, 'y_m': p.y, 'z_raw_m': p.z})
        elif topic == '/fusion/odometry':
            p=m.pose.pose.position
            specialized.setdefault('fusion', []).append({'t_s':t,'valid':True,'x_m':p.x,'y_m':p.y,
                'vx_map_mps':m.twist.twist.linear.x,'vy_map_mps':m.twist.twist.linear.y,
                'variance_x_m2':m.pose.covariance[0],'variance_y_m2':m.pose.covariance[7]})
        elif topic == '/fusion/status':
            if not m.valid:
                # Odometry itself is suppressed while invalid. Add an explicit
                # break even for a short (<plot-gap) invalid interval.
                specialized.setdefault('fusion', []).append({'t_s':t,'valid':False,
                    'x_m':math.nan,'y_m':math.nan,'vx_map_mps':math.nan,'vy_map_mps':math.nan,
                    'variance_x_m2':math.nan,'variance_y_m2':math.nan})
            specialized.setdefault('fusion_status', []).append({'t_s':t,'state':m.state,'valid':m.valid,
                'uwb_recent':m.uwb_recent,'flow_valid':m.flow_valid,'alignment_valid':m.alignment_valid,
                'uwb_age_ms':m.uwb_age_ms,'flow_age_ms':m.flow_age_ms,'attitude_age_ms':m.attitude_age_ms,
                'innovation_m':m.innovation_m,'accepted_uwb':m.accepted_uwb,'rejected_uwb':m.rejected_uwb,
                'flow_gaps':m.flow_gaps,'map_to_enu_yaw_rad':m.map_to_enu_yaw_rad})
            category=re.sub(r'[-+]?\d+(?:\.\d+)?','#',m.reason)
            signature=(m.state,category,m.valid,m.flow_valid,m.alignment_valid)
            if signature!=last_fusion:
                events.append({'t_s':t,'kind':'fusion','level':'ERROR' if m.state=='INVALID' else 'WARN' if m.state=='DEAD_RECKONING' else 'INFO','message':m.state+': '+m.reason})
            last_fusion=signature
        elif topic == '/navigation/heading_alignment':
            alignment = m
            events.append({'t_s': t, 'kind': 'heading_alignment', 'level': 'INFO' if m.valid else 'WARN',
                           'message': ('Existing alignment snapshot: ' if ns(m.header.stamp)<manifest['start_ros_ns']-100000000 else '')+m.reason, 'map_to_enu_yaw_rad': m.map_to_enu_yaw_rad})
        elif topic == '/fc/attitude':
            angles = euler(m.orientation, inverse) if m.valid else [math.nan]*3
            map_yaw = math.nan
            if alignment is not None and alignment.valid and monotonic is not None:
                elapsed_host = (monotonic-alignment.host_monotonic_ns)/1e6
                elapsed_device = (m.sample_time_us-alignment.device_time_us)/1000
                if m.sample_time_us >= alignment.device_time_us and abs(elapsed_host-elapsed_device) <= alignment.max_clock_skew_ms:
                    map_yaw = math.atan2(math.sin(math.pi/2-angles[2]-alignment.map_to_enu_yaw_rad),
                                         math.cos(math.pi/2-angles[2]-alignment.map_to_enu_yaw_rad))
            specialized.setdefault('attitude', []).append({'t_s': t, 'valid': m.valid,
                'roll_deg': math.degrees(angles[0]), 'pitch_deg': math.degrees(angles[1]),
                'yaw_fc_deg': math.degrees(angles[2]), 'yaw_map_deg': math.degrees(map_yaw)})
            previous = source_times.get(topic)
            if previous is not None and m.sample_time_us < previous:
                events.append({'t_s': t, 'kind': 'device_clock_reset', 'level': 'ERROR', 'message': topic})
            source_times[topic] = m.sample_time_us
        elif topic == '/fc/body_motion':
            specialized.setdefault('motion', []).append({'t_s': t, 'velocity_valid': m.velocity_valid,
                'forward_mps': m.forward_velocity_mps if m.velocity_valid else math.nan,
                'right_mps': m.right_velocity_mps if m.velocity_valid else math.nan,
                'height_up_m': m.height_up_m if m.height_valid else math.nan})
        elif topic in ('/fc/velocity_setpoint_body', '/fc/velocity_setpoint_sent'):
            name = 'body_command' if topic.endswith('_body') else 'wire_command'
            # Invalid inputs remain visible in raw CSV; plots explicitly mask
            # invalid body commands. Sent zeros are real transmitted zero values.
            keep = m.valid or name == 'wire_command'
            specialized.setdefault(name, []).append({'t_s': t, 'valid': m.valid,
                'x_mps': m.velocity.x if keep else math.nan, 'y_mps': m.velocity.y if keep else math.nan})
        elif topic == '/navigation/status':
            category = re.sub(r'[-+]?\d+(?:\.\d+)?', '#', m.reason)
            signature = (m.state, category, m.enabled, m.inputs_valid, m.goal_active, m.reached, m.feedback_assumed_zero)
            if last_nav is None or signature != last_nav['signature']:
                events.append({'t_s': t, 'kind': 'navigation', 'level': 'ERROR' if m.state == 'FAULT' else 'WARN' if m.state == 'INPUT_HOLD' else 'INFO', 'message': m.state+': '+m.reason, 'state_transition': last_nav is None or m.state != last_nav['state']})
            goal = (m.goal.x, m.goal.y) if m.goal_active else None
            if goal is not None and (last_nav is None or goal != last_nav['goal']):
                goal_id += 1
                events.append({'t_s': t, 'kind': 'goal_accepted', 'level': 'INFO', 'message': 'map XY=({}, {})'.format(*goal)})
            ex = ey = math.nan
            if goal is not None and last_pose and 0 <= t-last_pose[0] <= timeout_s:
                ex, ey = goal[0]-last_pose[1], goal[1]-last_pose[2]
            specialized.setdefault('navigation', []).append({'t_s': t, 'state': m.state, 'enabled': m.enabled,
                'inputs_valid': m.inputs_valid, 'goal_active': m.goal_active, 'goal_id': goal_id,
                'goal_x_m': m.goal.x if goal else math.nan, 'goal_y_m': m.goal.y if goal else math.nan,
                'error_x_m': ex, 'error_y_m': ey, 'distance_m': m.distance_m,
                'feedback_assumed_zero': m.feedback_assumed_zero,
                'actual_speed_mps': m.actual_speed_mps, 'position_age_ms': m.position_age_ms,
                'feedback_age_ms': m.feedback_age_ms, 'attitude_age_ms': m.attitude_age_ms,
                'heartbeat_age_ms': m.heartbeat_age_ms})
            last_nav = {'signature': signature, 'goal': goal, 'state': m.state}
        elif topic == '/rc/override':
            values = {'t_s': t, 'valid': m.valid}
            values.update({'ch{}'.format(i+1): value for i, value in enumerate(m.channels)})
            specialized.setdefault('rc', []).append(values)
        elif topic == '/diagnostics':
            for status in m.status:
                if status.name != 'mavlink_bridge':
                    continue
                values = {'t_s': t, 'message': status.message}
                for pair in status.values:
                    try:
                        values[pair.key] = float(pair.value)
                    except ValueError:
                        values[pair.key] = pair.value
                for key in ('rx_frames', 'tx_frames', 'velocity_frames', 'crc_or_signature_errors', 'tx_errors'):
                    derivative = math.nan
                    if previous_diag and key in values and key in previous_diag:
                        dt = t-previous_diag['t_s']
                        if dt > 0 and values[key] >= previous_diag[key]:
                            derivative = (values[key]-previous_diag[key])/dt
                        elif values[key] < previous_diag[key]:
                            events.append({'t_s': t, 'kind': 'counter_reset', 'level': 'WARN', 'message': key})
                    values[key+'_per_s'] = derivative
                specialized.setdefault('communication', []).append(values)
                previous_diag = values
        elif topic == '/experiment/mark':
            events.append({'t_s': t, 'kind': 'operator_note', 'level': 'INFO', 'message': m.data})
        elif topic == '/rosout' and m.level >= 30:
            # Transient-local logs older than the session are retained but marked
            # as history instead of being presented as new in-flight events.
            history = ns(m.stamp) < manifest['start_ros_ns']-100000000
            events.append({'t_s': t, 'kind': 'prior_ros_log' if history else 'ros_log',
                'level': 'ERROR' if m.level >= 40 else 'WARN', 'message': m.name+': '+m.msg})
    if interrupted:
        all_rows = [r for group in rows.values() for r in group]
        if all_rows:
            manifest['stop_steady_ns'] = manifest['start_steady_ns'] + int(max(r['t_s'] for r in all_rows)*1e9)
        else:
            manifest['stop_steady_ns'] = manifest['start_steady_ns']
    return manifest, data, params, rows, specialized, events, errors, counts


def plot_report(session, manifest, data, params, series, events, counts, errors):
    cache = session/'report'/'.matplotlib'
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR', str(cache))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    import numpy as np
    plt.rcParams.update({'figure.figsize': (11.7, 7.5), 'axes.grid': True, 'grid.alpha': .25,
                         'font.size': 10, 'savefig.dpi': 160, 'axes.spines.top': False, 'axes.spines.right': False})
    report = session/'report'; report.mkdir(exist_ok=True)
    control = params.get('position_controller_node', {})
    charts = []
    nav = series.get('navigation', [])
    def values(rows, key):
        return np.array([float(r.get(key, math.nan)) if isinstance(r.get(key, math.nan),(int,float)) else math.nan for r in rows])
    def line(ax, rows, key, label, gap=.5, **kwargs):
        if not rows:
            return
        t, y = values(rows, 't_s'), values(rows, key)
        y[~np.isfinite(y)] = np.nan
        # Insert NaNs rather than interpolating across dropped telemetry.
        broken_t=[];broken_y=[]
        for i in range(len(t)):
            if i and t[i]-t[i-1]>gap:
                broken_t.append(np.nan);broken_y.append(np.nan)
            broken_t.append(t[i]);broken_y.append(y[i])
        ax.plot(broken_t, broken_y, label=label, linewidth=1.1, **kwargs)
    def decorate(axes):
        for ax in np.array(axes, dtype=object).reshape(-1):
            ax.set_xlabel('Experiment time [s]')
            if ax.get_legend_handles_labels()[0]:
                ax.legend(loc='best', fontsize=8)
            for event in events:
                if event['kind']=='navigation' and event['level'] in ('WARN','ERROR'):
                    ax.axvline(event['t_s'], color='#d5524a', alpha=.15, linewidth=.8)
    def save(fig, name, title):
        fig.suptitle(title)
        fig.tight_layout(rect=(0,0,1,.96)); path=report/(name+'.png')
        fig.savefig(path); pdf.savefig(fig); charts.append((name,title,path));plt.close(fig)
    with PdfPages(report/'flight_report.pdf') as pdf:
        fig, axes=plt.subplots(1,2)
        position=series.get('position', [])
        if position:
            axes[0].plot(values(position,'x_m'),values(position,'y_m'),'.',markersize=2,alpha=.65,label='UWB map XY')
            axes[0].plot(position[0]['x_m'],position[0]['y_m'],'go',label='First sample')
        fused=series.get('fusion',[])
        if fused:
            axes[0].plot(values(fused,'x_m'),values(fused,'y_m'),'.',markersize=2,label='Independent fused XY')
        goals=[e for e in events if e['kind']=='goal_accepted']
        for ident in sorted({r['goal_id'] for r in nav if r['goal_active']}):
            group=[r for r in nav if r['goal_id']==ident and r['goal_active']]
            if group:axes[0].plot(group[0]['goal_x_m'],group[0]['goal_y_m'],'rx',markersize=9,label='Goal '+str(ident))
        scene=params.get('scene_visualizer_node',{})
        uwb=params.get('uwb_position_node',{})
        anchors=scene.get('anchor_positions_m',[])
        if isinstance(anchors,list) and len(anchors)==12 and scene.get('anchor_coordinates_configured',False):
            yaw=float(uwb.get('uwb_to_map_yaw_rad',0)); sign=float(uwb.get('uwb_y_sign',1));shift=uwb.get('uwb_to_map_translation_m',[0,0,0])
            for i,ident in enumerate(scene.get('anchor_ids',[0,1,2,3])):
                x,y=anchors[3*i],anchors[3*i+1]*sign
                x,y=math.cos(yaw)*x-math.sin(yaw)*y+shift[0],math.sin(yaw)*x+math.cos(yaw)*y+shift[1]
                axes[0].plot(x,y,'ks',markersize=5);axes[0].annotate('A'+str(ident),(x,y),xytext=(4,4),textcoords='offset points')
        axes[0].set_aspect('equal', adjustable='datalim');axes[0].set_xlabel('map X [m]');axes[0].set_ylabel('map Y [m]')
        if axes[0].get_legend_handles_labels()[0]: axes[0].legend(fontsize=8)
        line(axes[1],position,'x_m','Measured X');line(axes[1],position,'y_m','Measured Y')
        line(axes[1],nav,'goal_x_m','Goal X',linestyle='--');line(axes[1],nav,'goal_y_m','Goal Y',linestyle='--')
        axes[1].set_ylabel('Position [m]');decorate([axes[1]])
        save(fig,'trajectory','Map trajectory and accepted XY goals (XY projection)')
        fig, axes=plt.subplots(2,1,sharex=True)
        line(axes[0],nav,'error_x_m','X error');line(axes[0],nav,'error_y_m','Y error');axes[0].set_ylabel('Goal - position [m]')
        line(axes[1],nav,'distance_m','Controller distance');axes[1].axhline(float(control.get('arrival_position_tolerance_m',.05)),color='grey',linestyle='--',label='Arrival tolerance');axes[1].set_ylabel('Distance [m]');decorate(axes)
        save(fig,'position_error','Position tracking error (fresh pose only)')
        fig,axes=plt.subplots(2,1,sharex=True)
        for ax,key,cmd,label in zip(axes,['forward_mps','right_mps'],['x_mps','y_mps'],['Forward','Right']):
            line(ax,series.get('motion',[]),key,'Measured '+label)
            line(ax,series.get('body_command',[]),cmd,'Internal heading command',linestyle='--');ax.set_ylabel(label+' [m/s]')
        decorate(axes);save(fig,'velocity_tracking','Heading frame velocity: internal command vs measured forward/right')
        fig,axes=plt.subplots(2,1,sharex=True)
        for ax,key in zip(axes,['x_mps','y_mps']):
            line(ax,series.get('body_command',[]),key,'Internal FRD command')
            line(ax,series.get('wire_command',[]),key,'Sent ID84 field',linestyle='--');ax.set_ylabel(key)
        decorate(axes);save(fig,'sent_commands','Internal command and UDP ID84 fields (wire Y sign may be inverted)')
        fig,axes=plt.subplots(3,1,sharex=True)
        for ax,key in zip(axes,['roll_deg','pitch_deg','yaw_fc_deg']):
            line(ax,series.get('attitude',[]),key,key)
            if key=='yaw_fc_deg':line(ax,series.get('attitude',[]),'yaw_map_deg','map heading',linestyle='--')
            ax.set_ylabel('Angle [deg]')
        decorate(axes);save(fig,'attitude','FC attitude and valid-session map heading')
        fig,axes=plt.subplots(2,1,sharex=True)
        line(axes[0],series.get('motion',[]),'height_up_m','ID250 height up');axes[0].set_ylabel('Height [m]')
        line(axes[1],nav,'actual_speed_mps','Actual horizontal speed');axes[1].set_ylabel('Speed [m/s]');decorate(axes)
        save(fig,'height_speed','FC height and measured horizontal speed (invalid data left blank)')
        fig,axes=plt.subplots(2,1,sharex=True)
        for key in ['position_age_ms','attitude_age_ms','feedback_age_ms','heartbeat_age_ms']:line(axes[0],nav,key,key)
        axes[0].axhline(float(control.get('attitude_timeout_ms',1500)),color='grey',linestyle='--',label='Attitude timeout');axes[0].set_ylabel('Input age [ms]')
        state_names=list(dict.fromkeys(r['state'] for r in nav))
        if nav:
            line(axes[1], [{'t_s':r['t_s'], 'state_code':state_names.index(r['state'])} for r in nav], 'state_code', 'Navigation state', drawstyle='steps-post')
            axes[1].set_yticks(range(len(state_names)));axes[1].set_yticklabels(state_names)
        decorate(axes);save(fig,'input_health','Input freshness and controller state (heartbeat may be diagnostic only)')
        fig,axes=plt.subplots(2,1,sharex=True)
        comm=series.get('communication',[])
        for key in ['rx_frames_per_s','tx_frames_per_s','velocity_frames_per_s']:line(axes[0],comm,key,key,gap=2)
        for key in ['crc_or_signature_errors_per_s','tx_errors_per_s']:line(axes[1],comm,key,key,gap=2)
        axes[0].set_ylabel('Frames / s');axes[1].set_ylabel('Errors / s');decorate(axes)
        save(fig,'communication','MAVLink communication rates and error-counter changes')
        fig,axes=plt.subplots(4,2,sharex=True)
        for i,ax in enumerate(axes.flat):
            line(ax,series.get('rc',[]),'ch'+str(i+1),'CH'+str(i+1));ax.set_ylabel('PWM value')
        decorate(axes);save(fig,'rc_channels','Forwarded RC channels 1-8 (all 18 channels are in CSV)')
        fusion_status=series.get('fusion_status',[])
        if fusion_status:
            fused=series.get('fusion',[])
            fig,axes=plt.subplots(3,1,sharex=True)
            for ax,key in zip(axes[:2],['x_m','y_m']):
                line(ax,series.get('position',[]),key,'UWB')
                line(ax,fused,key,'Independent fused estimate',linestyle='--')
                ax.set_ylabel(key)
                for first,second in zip(fusion_status,fusion_status[1:]):
                    if first['state']=='DEAD_RECKONING' and second['t_s']-first['t_s']<.5:
                        ax.axvspan(first['t_s'],second['t_s'],color='#ffbd59',alpha=.16,linewidth=0)
            modes=list(dict.fromkeys(r['state'] for r in fusion_status))
            coded=[{'t_s':r['t_s'],'code':modes.index(r['state'])} for r in fusion_status]
            line(axes[2],coded,'code','Fusion mode',drawstyle='steps-post')
            axes[2].set_yticks(range(len(modes)));axes[2].set_yticklabels(modes)
            decorate(axes)
            save(fig,'fusion_comparison','Independent UWB/flow observer: amber = bounded dead reckoning; invalid odometry suppressed')

    return charts


def export_session(session, recover=False):
    session=Path(session).resolve()
    manifest,data,params,rows,series,events,errors,counts=read_session(session, recover)
    export=session/'csv';export.mkdir(exist_ok=True)
    for topic,values in rows.items():
        write_csv(export/(topic.strip('/').replace('/','__')+'.csv'),values)
    for key,values in series.items():write_csv(export/('derived_'+key+'.csv'),values)
    write_csv(session/'events.csv',events)
    event_text=''.join(json.dumps(e,ensure_ascii=False)+'\n' for e in events)
    (session/'events.jsonl').write_text(event_text,encoding='utf-8')
    eligible=[r for r in series.get('navigation',[]) if r['goal_active'] and r['enabled'] and r['inputs_valid'] and finite(r['error_x_m']) and finite(r['error_y_m'])]
    distances=[math.hypot(r['error_x_m'],r['error_y_m']) for r in eligible]
    distances=[x for x in distances if finite(x)]
    summary={'session':session.name,'experiment_label':manifest.get('label',''),'duration_s':max(0,(manifest.get('stop_steady_ns',0)-manifest['start_steady_ns'])/1e9),
             'message_counts':counts,'dropped_messages':manifest.get('dropped_messages',0),
             'recording_error':manifest.get('recording_error',''),'export_errors':errors,
             'active_tracking_samples':len(eligible),'position_rmse_m':math.sqrt(sum(x*x for x in distances)/len(distances)) if distances else None,
             'position_max_error_m':max(distances) if distances else None,
             'input_hold_events':sum(e['kind']=='navigation' and e['message'].startswith('INPUT_HOLD:') and e.get('state_transition',False) for e in events),
             'fault_events':sum(e['kind']=='navigation' and e['message'].startswith('FAULT:') and e.get('state_transition',False) for e in events),
             'notes':'Errors are sample-based over enabled, input-valid, active goals. No flight/ground classification. Raw NaNs retained; no gap interpolation. Sent ID84 confirms local UDP send, not onboard acceptance. dropped_messages counts local queue overflows only, not DDS/network loss.'}
    atomic_json(session/'summary.json',summary)
    charts=plot_report(session,manifest,data,params,series,events,counts,errors)
    images=[]
    for name,title,path in charts:
        encoded=base64.b64encode(path.read_bytes()).decode('ascii')
        images.append('<section><h2>'+html.escape(title)+'</h2><img src="data:image/png;base64,'+encoded+'" alt="'+html.escape(name)+'"></section>')
    coverage=''.join('<tr><td>'+html.escape(topic)+'</td><td>'+str(counts.get(topic,0))+'</td></tr>' for topic in manifest.get('topics',{}))
    event_rows=''.join('<tr><td>{:.3f}</td><td>{}</td><td>{}</td><td>{}</td></tr>'.format(e['t_s'],html.escape(e['level']),html.escape(e['kind']),html.escape(e['message'])) for e in events)
    config_html=html.escape(json.dumps(params,ensure_ascii=False,indent=2))
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>飞行实验报告</title>
<style>body{font:15px system-ui,sans-serif;color:#203047;max-width:1200px;margin:30px auto;padding:0 20px;background:#f4f6f9}section{background:white;padding:20px;margin:20px 0;border-radius:10px}img{width:100%;height:auto}table{border-collapse:collapse;width:100%}td,th{border:1px solid #dde3eb;padding:7px;text-align:left}pre{white-space:pre-wrap;overflow-wrap:anywhere}.notice{background:#fff0cb;padding:14px}h1,h2{color:#163858}</style>
<h1>飞行实验数据报告</h1>'''
    page+='<p>'+html.escape(manifest.get('label',''))+' · '+html.escape(session.name)+'</p><p>时长 {:.2f}s · 记录 {} 条消息 · 丢弃 {} 条</p>'.format(summary['duration_s'],sum(counts.values()),summary['dropped_messages'])
    page+='<p class="notice">曲线采用实验接收单调时间。内部速度为前/右方向，发送值可能对Y取反，二者分开绘制。无效数据留空；UWB Z不当作已验证高度。此报告不自动区分地面与飞行阶段。</p>'
    if errors or summary['recording_error'] or summary['dropped_messages']:
        page+='<p class="notice">数据完整性提示：'+html.escape(str(errors)+summary['recording_error']+' dropped='+str(summary['dropped_messages']))+'</p>'
    if series.get('fusion_status'):
        page+='<p class="notice">融合里程计是独立观察数据，未作为位置环输入。黄色区域表示短时光流推算；无效时不发布新的里程计。Z和机头姿态不由该XY里程计估计。</p>'
    page+=''.join(images)
    page+='<section><h2>状态、异常及人工标记</h2><table><tr><th>时间/s</th><th>级别</th><th>类型</th><th>内容</th></tr>'+event_rows+'</table></section>'
    page+='<section><h2>话题覆盖率（0表示未收到）</h2><table><tr><th>话题</th><th>消息数</th></tr>'+coverage+'</table></section>'
    page+='<section><h2>配置与运行参数快照</h2><pre>'+config_html+'</pre></section></html>'
    (session/'report.html').write_text(page,encoding='utf-8')
    print('报告：'+str(session/'report.html')); print('PDF：'+str(session/'report'/'flight_report.pdf'))
    return summary


def resolve_session(args):
    if args.session and args.session!='latest':return Path(args.session).resolve()
    root=Path(args.root).resolve()
    sessions=sorted((p for p in root.iterdir() if (p/'session.json').exists()),key=lambda p:load_json(p/'session.json').get('start_ros_ns',0)) if root.exists() else []
    if not sessions:raise ValueError('No experiment sessions under '+str(root))
    return sessions[-1]


def online(args):
    import rclpy
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy
    rclpy.init();node=rclpy.create_node('flight_log_cli')
    def read_status():
        result=[];qos=QoSProfile(depth=1,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        subscription=node.create_subscription(String,'/experiment/status',lambda m:result.append(m.data),qos)
        deadline=time.monotonic()+5
        while not result and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not result:raise RuntimeError('No recorder status received')
        # Drain retained status samples from older recorder builds as well.
        settle=time.monotonic()+.15
        while time.monotonic()<settle:rclpy.spin_once(node,timeout_sec=.01)
        node.destroy_subscription(subscription)
        return json.loads(result[-1])
    try:
        if args.command in ('start','stop'):
            client=node.create_client(Trigger,'/experiment/'+args.command)
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Experiment recorder unavailable; launch position_hold or experiment_logging first')
            future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=90)
            if not future.done():raise RuntimeError('Recorder response timed out; inspect /experiment/status')
            response=future.result();print(response.message)
            if not response.success:raise RuntimeError('Recording action did not complete: '+response.message)
        elif args.command=='status':
            print(json.dumps(read_status(),ensure_ascii=False,indent=2))
        else:
            current_status=read_status()
            if current_status.get('state')!='recording' or current_status.get('error'):
                raise RuntimeError('No active recording; run flight_log.sh start before adding a mark')
            pub=node.create_publisher(String,'/experiment/mark',10);deadline=time.monotonic()+3
            while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            if pub.get_subscription_count()==0:raise RuntimeError('No experiment marker subscriber')
            m=String();m.data=args.text;pub.publish(m)
            until=time.monotonic()+.2
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            print('已标记：'+args.text)
    finally:node.destroy_node();rclpy.shutdown()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    for command in ('start','stop','status'):sub.add_parser(command)
    mark=sub.add_parser('mark');mark.add_argument('text')
    export=sub.add_parser('export');export.add_argument('session',nargs='?',default='latest');export.add_argument('--root',default='log/experiments');export.add_argument('--recover',action='store_true',help='Export committed data after a crash; do not use while still recording')
    listing=sub.add_parser('list');listing.add_argument('--root',default='log/experiments')
    opening=sub.add_parser('open');opening.add_argument('session',nargs='?',default='latest');opening.add_argument('--root',default='log/experiments')
    args=parser.parse_args(argv)
    try:
        if args.command=='export':export_session(resolve_session(args), args.recover)
        elif args.command=='open':
            path=resolve_session(args)/'report.html'
            if not path.exists():raise ValueError('Report not available; stop recording or export first')
            subprocess.Popen(['xdg-open',str(path)])
        elif args.command=='list':
            root=Path(args.root)
            for path in sorted(root.glob('*/session.json')):
                m=load_json(path);print('{}  {}  {} messages'.format(path.parent,m.get('state'),m.get('written_messages',0)))
        else:online(args)
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print('flight_log: '+str(exc),file=sys.stderr);return 1
    return 0


if __name__=='__main__':sys.exit(main())
