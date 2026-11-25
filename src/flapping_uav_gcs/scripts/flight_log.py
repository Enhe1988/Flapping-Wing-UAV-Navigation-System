#!/usr/bin/env python3
"""Experiment control and offline Foxy rosbag SQLite -> CSV/PNG/PDF/HTML export.

Recorders are C++/rclcpp. This tool never sends flight commands; its online
commands address only /experiment/*. Offline export does not initialize DDS.
"""
import argparse
import base64
import csv
import fcntl
import html
import json
import math
import numbers
import os
from pathlib import Path
import pickle
import re
import sqlite3
import subprocess
import sys
import tempfile
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


class RawCsvSpool:
    """Bound raw-row RAM while preserving late-arriving variable-array columns."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.streams = {}
        self.fields = {}

    def __enter__(self):
        return self

    def __exit__(self, *unused):
        for stream in self.streams.values():
            stream.close()

    def append(self, topic, row):
        if topic not in self.streams:
            self.streams[topic] = tempfile.TemporaryFile(mode='w+b', dir=self.directory)
            self.fields[topic] = {}
        self.fields[topic].update(dict.fromkeys(row))
        # Only read our own unnamed temporary files. Binary spooling avoids
        # serializing the already-flattened nested JSON strings a second time.
        pickle.dump(row, self.streams[topic], protocol=pickle.HIGHEST_PROTOCOL)

    def finish(self):
        for topic, stream in self.streams.items():
            stream.seek(0)
            path = self.directory / (topic.strip('/').replace('/', '__') + '.csv')
            with path.open('w', newline='', encoding='utf-8-sig') as output:
                writer = csv.DictWriter(output, fieldnames=list(self.fields[topic]))
                writer.writeheader()
                while True:
                    try:
                        row = pickle.load(stream)
                    except EOFError:
                        break
                    writer.writerow(row)


def read_session(session, recover=False, raw_sink=None, progress=None):
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
    topic_health = {}
    previous_diag = None
    inverse = not bool(control.get('quaternion_body_to_ned', contract.get('quaternion_body_to_ned', True)))
    for topic, timestamp, monotonic, t, m in records(session, manifest, errors):
        counts[topic] = counts.get(topic, 0)+1
        row = {'t_s': t, 'receive_ros_ns': timestamp, 'receive_steady_ns': monotonic}
        row.update(flatten_fields(m))
        if raw_sink is None:
            rows.setdefault(topic, []).append(row)
        else:
            raw_sink.append(topic, row)
        health = topic_health.setdefault(topic, {'count': 0, 'first_t_s': t, 'last_t_s': t,
            'max_receive_gap_s': 0., 'nonmonotonic_intervals': 0})
        if health['count']:
            gap = t-health['last_t_s']
            health['max_receive_gap_s'] = max(health['max_receive_gap_s'], gap)
            health['nonmonotonic_intervals'] += int(gap < 0)
        health['count'] += 1
        health['last_t_s'] = t
        if progress and sum(counts.values()) % 5000 == 0:
            progress('reading', messages_read=sum(counts.values()))
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
            angles = ([math.nan,math.nan,m.yaw_rad] if getattr(m,'yaw_only',False) else
                      euler(m.orientation, inverse)) if m.valid else [math.nan]*3
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
    for health in topic_health.values():
        elapsed = health['last_t_s']-health['first_t_s']
        health['mean_receive_rate_hz'] = (health['count']-1)/elapsed if health['count'] > 1 and elapsed > 0 else None
    manifest = dict(manifest, topic_health=topic_health)
    if interrupted:
        end = max((h['last_t_s'] for h in topic_health.values()), default=0.)
        manifest['stop_steady_ns'] = manifest['start_steady_ns'] + int(max(0., end)*1e9)
    return manifest, data, params, rows, specialized, events, errors, counts


def plot_report(session, manifest, data, params, series, events, counts, errors, progress=None):
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
        if progress:
            progress('plotting', current_chart=name, completed_charts=len(charts))
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


def paper_csv_rows(session, name):
    path = session/name
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def localization_comparison(session, manifest, tables, progress=None):
    """Compare recorded UWB and parallel assisted estimates from ONE experiment."""
    import shutil
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    session = Path(session)
    out = session/'localization_comparison'
    out.mkdir(exist_ok=True)
    raw = tables.get('trajectory.csv', [])
    _, runtime_parameters=config_values(session)
    fusion_parameters=runtime_parameters.get('fusion_odometry_node',{})
    online_uwb_filter={'enabled':fusion_parameters.get('uwb_filter_enabled',False),
                       'tau_s':fusion_parameters.get('uwb_filter_tau_s')}
    if (session/'uwb_filtered.csv').exists():
        shutil.copyfile(session/'uwb_filtered.csv',out/'uwb_filtered_points.csv')
    sources = tables.get('position_sources.csv', [])
    reference = tables.get('reference.csv', [])
    fields = ['t_s', 'receive_ros_ns', 'source_ros_ns', 'x_m', 'y_m', 'valid', 'source',
              'uwb_healthy', 'flow_valid', 'raw_uwb_age_ms', 'reliable_uwb_age_ms',
              'flow_age_ms', 'innovation_m', 'uwb_syncs', 'odom_x_m', 'odom_y_m',
              'raw_uwb_x_m', 'raw_uwb_y_m']
    assisted = []
    for row in sources:
        # Keep the original numeric strings: no interpolation or float rounding.
        item = {key: row.get(key, '') for key in fields}
        item.update(x_m=row.get('selected_x_m', ''), y_m=row.get('selected_y_m', ''))
        assisted.append(item)
    with (out/'flow_compensated_points.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(assisted)
    for source, target, empty_fields in [
            ('trajectory.csv', 'uwb_raw_points.csv', ['t_s', 'receive_ros_ns', 'source_ros_ns', 'x_m', 'y_m', 'z_uwb_raw_m', 'frame_id']),
            ('reference.csv', 'reference_points.csv', ['t_s', 'goal_x_m', 'goal_y_m', 'goal_active']),
            ('position_sources.csv', 'source_status.csv', ['t_s', 'valid', 'source', 'uwb_healthy', 'flow_valid'])]:
        if (session/source).exists():
            shutil.copyfile(session/source, out/target)
        else:
            with (out/target).open('w', encoding='utf-8-sig', newline='') as stream:
                csv.writer(stream).writerow(empty_fields)

    def number(row, key):
        try: return float(row[key])
        except (KeyError, TypeError, ValueError): return math.nan
    def flag(row, key):
        return row.get(key) in ('1', 'true', 'True')
    def point_ok(row, x, y, validity=None):
        return (validity is None or flag(row, validity)) and all(
            finite(number(row, key)) for key in ['t_s', x, y])
    _, parameters = config_values(session)
    gap_s = parameters.get('fusion_odometry_node', {}).get('uwb_timeout_ms', 150)
    gap_s = float(gap_s)/1000 if isinstance(gap_s, (int, float)) and not isinstance(gap_s, bool) and gap_s > 0 else .15
    if not math.isfinite(gap_s): gap_s = .15
    selected_gap_s = .5  # Missing estimator publications must also break a curve.
    good_reference = [r for r in reference if flag(r, 'goal_active') and point_ok(r, 'goal_x_m', 'goal_y_m')]
    points = [(number(r, x), number(r, y)) for rows, x, y, valid in [
        (raw, 'x_m', 'y_m', None), (assisted, 'x_m', 'y_m', 'valid'),
        (good_reference, 'goal_x_m', 'goal_y_m', None)]
        for r in rows if point_ok(r, x, y, valid)]
    bounds = []
    for axis in [0, 1]:
        values = [p[axis] for p in points]
        low, high = (min(values), max(values)) if values else (-1., 1.)
        pad = max(.15, .08*(high-low))
        bounds.append([low-pad, high+pad])
    times = [number(r, 't_s') for rows in [raw, assisted, reference] for r in rows if finite(number(r, 't_s'))]
    time_limits = [min(0., min(times)), max(.1, max(times))] if times else [0., 1.]
    time_y_limits = [min(bounds[0][0], bounds[1][0]), max(bounds[0][1], bounds[1][1])]

    def spans(predicate):
        intervals = []; start = last = None
        for row in sources:
            t = number(row, 't_s')
            if not finite(t): continue
            if last is not None and (t < last or t-last > selected_gap_s):
                if start is not None: intervals.append([start, last])
                start = None
            if predicate(row):
                if start is None: start = t
            elif start is not None:
                intervals.append([start, t]); start = None
            last = t
        if start is not None: intervals.append([start, last])
        return intervals
    uwb_bad = spans(lambda r: r.get('uwb_healthy') in ('0', 'false', 'False'))
    flow_used = spans(lambda r: flag(r, 'valid') and r.get('source') == 'FLOW_ODOMETRY')
    continuous = manifest.get('flight_mode', 'waypoint') in ('line', 'figure8')
    synthetic = 'synthetic' in manifest.get('label', '').lower()
    source_counts = {key: sum(point_ok(r, 'x_m', 'y_m', 'valid') and r.get('source') == key for r in assisted)
                     for key in ['UWB', 'FLOW_ODOMETRY', 'UWB_BLEND']}

    def coordinates(rows, x, y, valid=None, predicate=None, max_gap=.5):
        xs = []; ys = []; previous = None
        for row in rows:
            t = number(row, 't_s')
            if previous is not None and (not finite(t) or t < previous or t-previous > max_gap):
                xs.append(math.nan); ys.append(math.nan)
            ok = point_ok(row, x, y, valid) and (predicate is None or predicate(row))
            xs.append(number(row, x) if ok else math.nan)
            ys.append(number(row, y) if ok else math.nan)
            previous = t if finite(t) else None
        return xs, ys

    names = ['uwb_raw_trajectory', 'flow_compensated_trajectory']
    for index, (name, rows) in enumerate(zip(names, [raw, assisted])):
        if progress: progress('plotting_localization_comparison', current_chart=name)
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
        title = 'Raw UWB position estimate' if index == 0 else 'UWB + optical-flow position estimate'
        prefix = 'SYNTHETIC VALIDATION — ' if synthetic else ''
        if index == 0:
            x, y = coordinates(rows, 'x_m', 'y_m', max_gap=gap_s)
            axes[0].plot(x, y, '.-', markersize=2, linewidth=.7, color='#b54835', label='Recorded raw UWB')
        else:
            colors = {'UWB': '#2471a3', 'FLOW_ODOMETRY': '#d68910', 'UWB_BLEND': '#239b56'}
            for source, color in colors.items():
                if source_counts[source]:
                    x, y = coordinates(rows, 'x_m', 'y_m', 'valid', lambda r, s=source: r.get('source') == s)
                    axes[0].plot(x, y, '.-', markersize=2, linewidth=.7, color=color, label=source)
        if continuous:
            x, y = coordinates(reference, 'goal_x_m', 'goal_y_m', 'goal_active', lambda r: flag(r, 'trajectory_running'))
            axes[0].plot(x, y, '--', linewidth=1, color='#333333', label='Executed desired trajectory')
        else:
            targets = list(dict.fromkeys((number(r, 'goal_x_m'), number(r, 'goal_y_m')) for r in good_reference))
            if targets: axes[0].plot([p[0] for p in targets], [p[1] for p in targets], 'kx', label='Desired targets')
        plotted = sum(point_ok(r, 'x_m', 'y_m', 'valid' if index else None) for r in rows)
        if not plotted: axes[0].text(.5, .5, 'No valid recorded samples', ha='center', transform=axes[0].transAxes)
        axes[0].set_xlim(*bounds[0]); axes[0].set_ylim(*bounds[1]); axes[0].set_aspect('equal', adjustable='box')
        axes[0].set_xlabel('map X [m]'); axes[0].set_ylabel('map Y [m]')
        for key, label in [('x_m', 'X'), ('y_m', 'Y')]:
            # Reuse the 2D break logic so invalidity and gaps are not bridged.
            ts, values = coordinates(rows, 't_s', key, 'valid' if index else None,
                                     max_gap=selected_gap_s if index else gap_s)
            axes[1].plot(ts, values, linewidth=.9, label='Recorded '+label)
        for key, label in [('goal_x_m', 'X'), ('goal_y_m', 'Y')]:
            ts, values = coordinates(reference, 't_s', key, 'goal_active')
            axes[1].plot(ts, values, '--', linewidth=.8, drawstyle='default' if continuous else 'steps-post', label='Desired '+label)
        for i, (left, right) in enumerate(uwb_bad):
            axes[1].axvspan(left, right, color='#c0392b', alpha=.13, label='UWB not accepted' if i == 0 else None)
        axes[1].set_xlim(*time_limits); axes[1].set_ylim(*time_y_limits)
        axes[1].set_xlabel('Time since recording start [s]'); axes[1].set_ylabel('Position estimate [m]')
        for ax in axes:
            ax.grid(alpha=.25)
            if ax.get_legend_handles_labels()[0]: ax.legend(fontsize=8)
        fig.suptitle(prefix+title)
        fig.text(.5, .015, 'Same experiment; unfiltered samples; shared axes. Positions are estimates, not independent ground truth.', ha='center', fontsize=8)
        fig.tight_layout(rect=(0, .045, 1, .95))
        fig.savefig(out/(name+'.png'), dpi=300); fig.savefig(out/(name+'.pdf')); plt.close(fig)
    metadata = {
        'comparison_type': 'same_experiment_position_estimates', 'session': str(session.resolve()),
        'synthetic_validation': synthetic, 'offline_filtering': False, 'resampled': False,
        'online_uwb_filter':online_uwb_filter,
        'filtered_uwb_points_source':'uwb_filtered.csv (/uwb/position_filtered, accepted online low-pass fixes)',
        'xy_limits_m': bounds, 'time_limits_s': time_limits, 'time_position_limits_m': time_y_limits,
        'raw_uwb_gap_threshold_s': gap_s, 'assisted_gap_threshold_s': selected_gap_s,
        'raw_point_rows': len(raw), 'compensated_point_rows': len(assisted),
        'valid_compensated_point_rows': sum(point_ok(r, 'x_m', 'y_m', 'valid') for r in assisted),
        'valid_source_counts': source_counts, 'flow_fallback_observed': source_counts['FLOW_ODOMETRY'] > 0,
        'uwb_unhealthy_intervals_s': uwb_bad, 'flow_fallback_intervals_s': flow_used,
        'assistance_used_by_controller': manifest.get('flow_assistance_enabled'), 'plots': names,
        'raw_points_source': 'trajectory.csv (/uwb/position, after the UWB driver map conversion)',
        'compensated_points_source': 'position_sources.csv selected_x_m/selected_y_m (/localization/position)',
        'notes': 'Two estimates from the SAME experiment, not separate assisted/unassisted flights. '
                 'Raw UWB is the driver output; fixes rejected by the driver cannot be recovered here. '
                 'Compensated points are recorded online selector outputs, not an offline reconstruction. '
                 'When enabled, online UWB smoothing affects the compensated output even without flow fallback; '
                 'use uwb_filtered.csv and same-filter assisted/unassisted flights to separate these effects. '
                 'Invalid points stay in CSV with their flags but are not plotted; gaps remain gaps. '
                 'UWB-not-accepted shading is algorithm status, not a measured physical occlusion label. '
                 'Full precision numeric strings and independent timestamps are retained; original CSV files are untouched. '
                 'No guaranteed improvement: verify flow_fallback_observed and the actual recorded curves.'}
    atomic_json(out/'comparison.json', metadata)
    return metadata


def uwb_filter_comparison(session, tables, progress=None):
    """Two views of recorded raw/online-filtered UWB, with identical axes.

    No offline smoothing, resampling, rejection or modification of source CSV.
    """
    import matplotlib.pyplot as plt
    session = Path(session)
    raw = tables.get('trajectory.csv', [])
    filtered = tables.get('uwb_filtered.csv', [])
    reference = tables.get('reference.csv', [])
    metadata = {'comparison_type': 'same_experiment_raw_and_online_filtered_uwb',
                'raw_csv': 'trajectory.csv', 'filtered_csv': 'uwb_filtered.csv',
                'reference_csv': 'reference.csv', 'offline_filtering': False,
                'resampled': False, 'plots': []}
    if not raw or not filtered:
        metadata['missing_data'] = [name for name, rows in
            [('trajectory.csv', raw), ('uwb_filtered.csv', filtered)] if not rows]
        return metadata

    def number(row, key):
        try: return float(row[key])
        except (KeyError, TypeError, ValueError): return math.nan
    def flag(row, key):
        return row.get(key) in ('1', 'true', 'True')
    _, parameters = config_values(session)
    config = parameters.get('fusion_odometry_node', {})
    gap_ms = config.get('uwb_timeout_ms', 300)
    gap_s = gap_ms / 1000 if isinstance(gap_ms, (int, float)) and not isinstance(gap_ms, bool) and finite(gap_ms) and gap_ms > 0 else .3
    metadata['online_filter_enabled'] = config.get('uwb_filter_enabled')
    metadata['online_filter_tau_s'] = config.get('uwb_filter_tau_s')
    metadata['gap_break_s'] = gap_s
    active = [r for r in reference if flag(r, 'goal_active')]
    points = [(number(r, x), number(r, y)) for rows, x, y in
              [(raw, 'x_m', 'y_m'), (filtered, 'x_m', 'y_m'),
               (active, 'goal_x_m', 'goal_y_m')]
              for r in rows if finite(number(r, x)) and finite(number(r, y))]
    times = [number(r, 't_s') for rows in (raw, filtered, reference)
             for r in rows if finite(number(r, 't_s'))]
    if not points or not times:
        metadata['missing_data'] = ['finite XY/time samples']
        return metadata
    bounds = []
    for axis in (0, 1):
        low, high = min(p[axis] for p in points), max(p[axis] for p in points)
        padding = max(.1, (high - low) * .05)
        bounds.append([low - padding, high + padding])
    time_bounds = [min(times), max(times)]
    if time_bounds[1] <= time_bounds[0]: time_bounds[1] = time_bounds[0] + 1
    metadata.update(xy_limits_m=bounds, time_limits_s=time_bounds)

    def curve(rows, x, y, validity=None, gap=gap_s):
        xs, ys, previous = [], [], None
        for row in rows:
            stamp, a, b = number(row, 't_s'), number(row, x), number(row, y)
            good = all(finite(v) for v in (stamp, a, b)) and (validity is None or flag(row, validity))
            if not good:
                xs.append(math.nan); ys.append(math.nan); previous = None
                continue
            if previous is not None and (stamp <= previous or stamp - previous > gap):
                xs.append(math.nan); ys.append(math.nan)
            xs.append(a); ys.append(b); previous = stamp
        return xs, ys

    running = [dict(r, plot_reference=str(int(flag(r, 'goal_active') and flag(r, 'trajectory_running'))))
               for r in reference]
    targets = []
    for row in active:
        point = (number(row, 'goal_x_m'), number(row, 'goal_y_m'))
        if not flag(row, 'trajectory_running') and all(finite(v) for v in point) and (not targets or targets[-1] != point):
            targets.append(point)
    report = session / 'report'; report.mkdir(exist_ok=True)
    for rows, name, label in [(raw, 'trajectory_uwb_raw', 'Raw UWB'),
                              (filtered, 'trajectory_uwb_filtered', 'Online filtered UWB')]:
        if progress: progress('plotting', current_chart=name)
        fig = plt.figure(figsize=(11, 5.6))
        grid = fig.add_gridspec(2, 2)
        xy = fig.add_subplot(grid[:, 0])
        tx = fig.add_subplot(grid[0, 1]); ty = fig.add_subplot(grid[1, 1], sharex=tx)
        xy.plot(*curve(rows, 'x_m', 'y_m'), color='tab:blue', linewidth=.8,
                marker='.', markersize=1.5, label=label)
        xy.plot(*curve(running, 'goal_x_m', 'goal_y_m', 'plot_reference', .5),
                '--', color='tab:red', linewidth=1.2, label='Executed desired XY trajectory')
        if targets:
            xy.plot([p[0] for p in targets], [p[1] for p in targets], 'rx', markersize=6, label='Discrete targets')
        xy.set_xlim(bounds[0]); xy.set_ylim(bounds[1]); xy.set_aspect('equal', adjustable='box')
        xy.set_xlabel('map X [m]'); xy.set_ylabel('map Y [m]'); xy.legend(fontsize=8)
        for ax, key, goal, axis in [(tx, 'x_m', 'goal_x_m', 0), (ty, 'y_m', 'goal_y_m', 1)]:
            ax.plot(*curve(rows, 't_s', key), color='tab:blue', linewidth=.8, label=label)
            ax.plot(*curve(reference, 't_s', goal, 'goal_active', .5), '--',
                    color='tab:red', linewidth=1.2, label='Desired '+('X' if axis == 0 else 'Y'))
            ax.set_xlim(time_bounds); ax.set_ylim(bounds[axis]); ax.legend(fontsize=8)
            ax.set_ylabel('map '+('X' if axis == 0 else 'Y')+' [m]')
        ty.set_xlabel('Time since recording start [s]')
        fig.suptitle(label+' and recorded desired position')
        fig.tight_layout(rect=(0, 0, 1, .95))
        try:
            fig.savefig(report/(name+'.png'), dpi=300)
            fig.savefig(report/(name+'.pdf'))
        finally: plt.close(fig)
        metadata['plots'].append(name)
    return metadata


def paper_export(session, manifest, recover=False, progress=None):
    """Plot unfiltered direct CSV recordings; never rewrites the source tables."""
    tables = {name: paper_csv_rows(session, name) for name in manifest['files']}
    if progress:
        progress('reading_csv', messages_read=sum(len(rows) for rows in tables.values()))
    trajectory=tables.get('trajectory.csv',[])
    filtered_uwb=tables.get('uwb_filtered.csv',[])
    used=tables.get('control_position.csv',trajectory)
    has_selected='control_position.csv' in tables
    reference=tables.get('reference.csv',[])
    desired=tables.get('velocity_desired.csv',[])
    actual=tables.get('velocity_actual.csv',[])
    attitude=tables.get('attitude.csv',[])
    angle_desired=tables.get('attitude_desired.csv',[])
    angle_sent=tables.get('attitude_sent.csv',[])
    angle_pid=tables.get('attitude_pid.csv',[])
    continuous_reference=manifest.get('flight_mode','waypoint') in ('line','figure8')
    report=session/'report';report.mkdir(exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR',str(report/'.matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.22,
                         'axes.spines.top':False,'axes.spines.right':False})
    def number(row,key):
        try:return float(row[key])
        except (KeyError,TypeError,ValueError):return math.nan
    def valid(row,key):
        return row.get(key) in ('1','True','true')
    def values(rows,key,flag=None):
        return np.array([number(r,key) if flag is None or valid(r,flag) else math.nan for r in rows])
    def line(ax,rows,key,label,flag=None,step=False,**style):
        t,y=values(rows,'t_s'),values(rows,key,flag)
        tx=[];vy=[]
        for i in range(len(t)):
            if i and (t[i]-t[i-1]>.5 or t[i]<t[i-1]):tx.append(math.nan);vy.append(math.nan)
            tx.append(t[i]);vy.append(y[i] if finite(y[i]) else math.nan)
        ax.plot(tx,vy,label=label,drawstyle='steps-post' if step else 'default',linewidth=1.,**style)
    charts=[]
    def save(fig,name,title):
        if progress:progress('plotting',current_chart=name,completed_charts=len(charts))
        fig.suptitle(title);fig.tight_layout(rect=(0,0,1,.95))
        fig.savefig(report/(name+'.png'),dpi=300)
        fig.savefig(report/(name+'.pdf'))
        charts.append(name);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4.4))
    axes[0].plot(values(trajectory,'x_m'),values(trajectory,'y_m'),'.',markersize=1.2,alpha=.35,color='grey',label='Raw UWB')
    if filtered_uwb:
        axes[0].plot(values(filtered_uwb,'x_m'),values(filtered_uwb,'y_m'),'.',markersize=1.1,alpha=.65,color='tab:green',label='Online filtered UWB')
    axes[0].plot(values(used,'x_m','valid' if has_selected else None),values(used,'y_m','valid' if has_selected else None),'.',markersize=1.5,label='Controller position estimate')
    if continuous_reference:
        axes[0].plot([number(r,'goal_x_m') if valid(r,'trajectory_running') else math.nan for r in reference],
                     [number(r,'goal_y_m') if valid(r,'trajectory_running') else math.nan for r in reference],
                     '--',color='red',linewidth=1.,label='Executed desired XY trajectory')
    targets=[]
    for row in reference:
        target=(number(row,'goal_x_m'),number(row,'goal_y_m'))
        if valid(row,'goal_active') and not valid(row,'trajectory_running') and all(finite(v) for v in target) and (not targets or target!=targets[-1]):targets.append(target)
    if targets:
        axes[0].plot([p[0] for p in targets],[p[1] for p in targets],'rx',markersize=7,label='Discrete targets')
    axes[0].set_xlabel('map X [m]');axes[0].set_ylabel('map Y [m]');axes[0].set_aspect('equal',adjustable='datalim');axes[0].legend()
    for key,goal,label in [('x_m','goal_x_m','X'),('y_m','goal_y_m','Y')]:
        line(axes[1],used,key,'Controller '+label,'valid' if has_selected else None)
        line(axes[1],reference,goal,'Desired '+label,'goal_active',step=not continuous_reference,linestyle='--')
    axes[1].set_xlabel('Time since recording start [s]');axes[1].set_ylabel('Position [m]');axes[1].legend()
    save(fig,'trajectory','Desired position, raw UWB and controller position estimate')
    filter_comparison = uwb_filter_comparison(session, tables, progress)
    charts.extend(filter_comparison['plots'])
    yaw_only=any(valid(row,'yaw_only') for row in attitude)
    fig,axes=plt.subplots(2,1,sharex=True,figsize=(9,5.5))
    if angle_desired:
        for ax,key,label in zip(axes,['pitch_deg','roll_deg'],['Pitch','Roll']):
            line(ax,angle_desired,key,'Controller target','valid',linestyle='--')
            line(ax,angle_sent,key,'UDP sent')
            ax.axhline(30,color='grey',linestyle=':');ax.axhline(-30,color='grey',linestyle=':')
            ax.set_ylabel(label+' [deg]');ax.legend()
        axes[-1].set_xlabel('Time since recording start [s]')
        save(fig,'attitude_targets','Position-loop attitude targets and transmitted angles')
    else:
        for ax,key,label in zip(axes,['forward_mps','right_mps'],['Forward','Right']):
            line(ax,desired,key,'Desired','valid',linestyle='--')
            if actual:
                line(ax,actual,key,'Actual','velocity_valid')
            elif yaw_only:
                ax.text(.02,.90,'Measured velocity not transmitted (compact mode)',transform=ax.transAxes,fontsize=9)
            ax.set_ylabel(label+' [m/s]');ax.legend()
        axes[-1].set_xlabel('Time since recording start [s]')
        save(fig,'velocity','Horizontal heading-frame velocity tracking')
    if angle_pid:
        fig,axes=plt.subplots(3,1,sharex=True,figsize=(10,8))
        for ax,axis in zip(axes[:2],['x','y']):
            for term in ['p','i','d']:
                line(ax,angle_pid,term+'_map_'+axis+'_deg',term.upper(),'valid')
            ax.set_ylabel('map '+axis.upper()+' demand [deg]');ax.legend()
        for axis in ['x','y']:
            line(axes[2],angle_pid,'uwb_v'+axis+'_est_mps','UWB estimate '+axis.upper(),'velocity_estimate_valid')
        axes[2].set_ylabel('Estimated velocity [m/s]');axes[2].legend()
        axes[-1].set_xlabel('Time since recording start [s]')
        save(fig,'attitude_pid','Position-to-attitude PID before body rotation and output limits')
    fig,axes=plt.subplots(3,2,sharex=True,figsize=(10,7.5))
    for i,(angle,rate,label) in enumerate(zip(['roll_deg','pitch_deg','yaw_map_deg'],['p_rad_s','q_rad_s','r_rad_s'],['Roll','Pitch','Yaw'])):
        line(axes[i,0],attitude,angle,label,'attitude_valid')
        line(axes[i,1],attitude,rate,['p','q','r'][i],'angular_velocity_valid')
        axes[i,0].set_ylabel(label+' [deg]');axes[i,1].set_ylabel(['p','q','r'][i]+' [rad/s]')
        if yaw_only:
            if i<2:axes[i,0].text(.02,.85,'Not transmitted',transform=axes[i,0].transAxes,fontsize=9)
            axes[i,1].text(.02,.85,'Not transmitted',transform=axes[i,1].transAxes,fontsize=9)
    axes[-1,0].set_xlabel('Time since recording start [s]');axes[-1,1].set_xlabel('Time since recording start [s]')
    save(fig,'attitude','Unfiltered Euler angles and reported body angular rates')
    comparison = localization_comparison(session, manifest, tables, progress)
    expected=['trajectory.csv','reference.csv','attitude.csv']+(['attitude_desired.csv','attitude_sent.csv'] if angle_desired else ['velocity_desired.csv','velocity_actual.csv'])
    missing=[name for name in expected if not tables.get(name)]
    if has_selected and not used:missing.append('control_position.csv')
    mismatches=[]
    if not recover:
        mismatches=[name for name,meta in manifest['files'].items() if meta['rows']!=len(tables[name])]
    summary={'format':'paper_csv_v1','session':session.name,'raw_rows':{name:len(rows) for name,rows in tables.items()},
             'missing_data':missing,'row_count_mismatches':mismatches,'dropped_rows':manifest.get('dropped_messages',0),
             'recording_error':manifest.get('recording_error',''),'charts':charts,'unfiltered':True,
             'flow_assistance_enabled':manifest.get('flow_assistance_enabled'),
             'online_uwb_filter':comparison['online_uwb_filter'],
             'uwb_filter_comparison':filter_comparison,
             'flight_mode':manifest.get('flight_mode','waypoint'),
             'localization_comparison': {'metadata': 'localization_comparison/comparison.json',
                 'flow_fallback_observed': comparison['flow_fallback_observed'], 'plots': comparison['plots']},
             'notes':'Independent source timestamps retained. No offline resampling, smoothing or fabricated zeros. Controller positions include online source selection/blending; raw UWB is retained separately. p/q/r are reported body rates, not Euler derivatives. Desired positions are actual executed references: point targets or analytic horizontal trajectories according to flight_mode.'}
    atomic_json(session/'summary.json',summary)
    if progress:progress('writing_html',completed_charts=len(charts))
    page='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>论文实验记录</title><body style="max-width:1100px;margin:auto;font-family:sans-serif">'
    page+='<h1>论文实验记录</h1><p>'+html.escape(session.name)+'</p><p>原始CSV未经离线滤波；角度单位deg，机体角速度单位rad/s。期望位置为导航节点实际接受的目标。Raw UWB为原始定位输出，Controller position estimate为控制器实际采用的估计，不是独立真值。</p>'
    page+='<p>滤波前后对照：trajectory_uwb_raw为原始UWB，trajectory_uwb_filtered为实验中实际记录的在线滤波UWB。两图使用相同坐标和时间范围及相同期望参考，不进行额外离线滤波；原始点分别保存在trajectory.csv与uwb_filtered.csv，参考保存在reference.csv。</p>'
    if filter_comparison.get('missing_data'):
        page+='<p>未生成UWB滤波前后两图：缺少'+html.escape(', '.join(filter_comparison['missing_data']))+'，不使用离线重算数据代替在线记录。</p>'
    if missing or mismatches or summary['recording_error'] or summary['dropped_rows']:
        page+='<p>数据提示：'+html.escape(json.dumps({k:summary[k] for k in ['missing_data','row_count_mismatches','recording_error','dropped_rows']},ensure_ascii=False))+'</p>'
    if manifest['state']=='recording':page+='<p>这是异常中断后从已落盘CSV恢复的报告，可能缺少尾部数据。</p>'
    for name in charts:page+='<p><a href="report/'+name+'.pdf">'+name+' PDF</a></p><img style="width:100%" src="report/'+name+'.png">'
    page+='<h2>同次实验：原始UWB与光流补偿定位</h2><p>两张图使用相同坐标和时间范围，原始点未经离线滤波。补偿曲线为实时定位选择器的记录，不是另一架次的无辅助飞行轨迹。着色区间表示算法未接受UWB，断流不补点。</p>'
    if comparison['online_uwb_filter']['enabled']:
        page+='<p>本次启用在线UWB低通，时间常数为{}秒。补偿曲线同时包含在线滤波与可能的光流接管效果；仅凭原始/补偿两张图不能把全部改善归因于光流。uwb_filtered.csv保存滤波后的独立UWB参考点，原始trajectory.csv保持不变。</p>'.format(html.escape(str(comparison['online_uwb_filter']['tau_s'])))
    page+='<p>本次定位选择器的有效光流接管输出样本：{}；控制器辅助开关：{}。</p>'.format(comparison['valid_source_counts']['FLOW_ODOMETRY'], html.escape(str(comparison['assistance_used_by_controller'])))
    if not comparison['flow_fallback_observed']:page+='<p>本次未记录有效光流接管样本，不能据此说明接管改善。</p>'
    for name in comparison['plots']:
        page+='<p><a href="localization_comparison/'+name+'.pdf">'+name+' PDF</a></p><img style="width:100%" src="localization_comparison/'+name+'.png">'
    page+='<p>对照原始点：</p><ul>'+''.join('<li><a href="localization_comparison/'+name+'">'+name+'</a></li>' for name in ['uwb_raw_points.csv','flow_compensated_points.csv','reference_points.csv','source_status.csv','comparison.json'])+'</ul>'
    page+='<p>CSV文件：</p><ul>'+''.join('<li><a href="'+name+'">'+name+'</a></li>' for name in tables)+'</ul></body></html>'
    (session/'report.html').write_text(page,encoding='utf-8')
    print('论文CSV与图表已保存：'+str(session))
    return summary


def _export_session(session, recover=False, progress=None):
    manifest=load_json(session/'session.json')
    if manifest.get('format')=='paper_csv_v1':
        return paper_export(session,manifest,recover,progress)
    export=session/'csv';export.mkdir(exist_ok=True)
    with RawCsvSpool(export) as spool:
        manifest,data,params,rows,series,events,errors,counts=read_session(session, recover, spool, progress)
        if progress:
            progress('writing_csv', messages_read=sum(counts.values()))
        spool.finish()
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
             'topic_health':manifest['topic_health'],
             'missing_core_topics':[topic for topic in ('/uwb/position','/fc/attitude','/fc/body_motion') if not counts.get(topic)],
             'notes':'Errors are sample-based over enabled, input-valid, active goals. No flight/ground classification. Raw NaNs retained; no gap interpolation. Sent ID84 confirms local UDP send, not onboard acceptance. dropped_messages counts local queue overflows only, not DDS/network loss.'}
    atomic_json(session/'summary.json',summary)
    charts=plot_report(session,manifest,data,params,series,events,counts,errors,progress)
    if progress:
        progress('writing_html', completed_charts=len(charts))
    images=[]
    for name,title,path in charts:
        encoded=base64.b64encode(path.read_bytes()).decode('ascii')
        images.append('<section><h2>'+html.escape(title)+'</h2><img src="data:image/png;base64,'+encoded+'" alt="'+html.escape(name)+'"></section>')
    coverage=''
    for topic in manifest.get('topics',{}):
        health=manifest['topic_health'].get(topic,{})
        rate=health.get('mean_receive_rate_hz')
        gap=health.get('max_receive_gap_s')
        coverage+='<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>'.format(
            html.escape(topic),counts.get(topic,0),'{:.2f}'.format(rate) if rate is not None else '—',
            '{:.3f}'.format(gap) if gap is not None else '—')
    event_rows=''.join('<tr><td>{:.3f}</td><td>{}</td><td>{}</td><td>{}</td></tr>'.format(e['t_s'],html.escape(e['level']),html.escape(e['kind']),html.escape(e['message'])) for e in events)
    config_html=html.escape(json.dumps(params,ensure_ascii=False,indent=2))
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>飞行实验报告</title>
<style>body{font:15px system-ui,sans-serif;color:#203047;max-width:1200px;margin:30px auto;padding:0 20px;background:#f4f6f9}section{background:white;padding:20px;margin:20px 0;border-radius:10px}img{width:100%;height:auto}table{border-collapse:collapse;width:100%}td,th{border:1px solid #dde3eb;padding:7px;text-align:left}pre{white-space:pre-wrap;overflow-wrap:anywhere}.notice{background:#fff0cb;padding:14px}h1,h2{color:#163858}</style>
<h1>飞行实验数据报告</h1>'''
    page+='<p>'+html.escape(manifest.get('label',''))+' · '+html.escape(session.name)+'</p><p>时长 {:.2f}s · 记录 {} 条消息 · 丢弃 {} 条</p>'.format(summary['duration_s'],sum(counts.values()),summary['dropped_messages'])
    page+='<p class="notice">曲线采用实验接收单调时间。内部速度为前/右方向，发送值可能对Y取反，二者分开绘制。无效数据留空；UWB Z不当作已验证高度。此报告不自动区分地面与飞行阶段。</p>'
    if errors or summary['recording_error'] or summary['dropped_messages']:
        page+='<p class="notice">数据完整性提示：'+html.escape(str(errors)+summary['recording_error']+' dropped='+str(summary['dropped_messages']))+'</p>'
    if summary['missing_core_topics']:
        page+='<p class="notice">本次未收到以下定位/飞控数据：'+html.escape(', '.join(summary['missing_core_topics']))+'。请按实验目的核对；空白曲线不代表数值为零。</p>'
    if series.get('fusion_status'):
        page+='<p class="notice">融合里程计是独立观察数据，未作为位置环输入。黄色区域表示短时光流推算；无效时不发布新的里程计。Z和机头姿态不由该XY里程计估计。</p>'
    page+=''.join(images)
    page+='<section><h2>状态、异常及人工标记</h2><table><tr><th>时间/s</th><th>级别</th><th>类型</th><th>内容</th></tr>'+event_rows+'</table></section>'
    page+='<section><h2>话题接收情况（0表示未收到）</h2><p>频率和间隔为电脑接收统计，不是源发送频率或网络丢包率。</p><table><tr><th>话题</th><th>消息数</th><th>平均接收Hz</th><th>最大接收间隔/s</th></tr>'+coverage+'</table></section>'
    page+='<section><h2>配置与运行参数快照</h2><pre>'+config_html+'</pre></section></html>'
    (session/'report.html').write_text(page,encoding='utf-8')
    print('报告：'+str(session/'report.html')); print('PDF：'+str(session/'report'/'flight_report.pdf'))
    return summary


def export_session(session, recover=False):
    session=Path(session).resolve()
    manifest=load_json(session/'session.json')
    if 'start_steady_ns' not in manifest:
        raise ValueError('Not an experiment directory: '+str(session))
    if manifest.get('state')=='recording' and not recover:
        raise ValueError('Session is still recording. Stop it before export; raw data is untouched.')
    # A manual export and the recorder's automatic export must not overwrite
    # the same CSV/PDF/HTML concurrently. A stale lock file is harmless.
    with (session/'.export.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Report export already running for '+str(session))
        status={'state':'exporting','stage':'reading','started_ros_ns':time.time_ns()}
        def progress(stage, **values):
            status.update(values, stage=stage, updated_ros_ns=time.time_ns())
            atomic_json(session/'export_status.json',status)
        progress('reading')
        try:
            summary=_export_session(session,recover,progress)
        except BaseException as exc:
            status.update(state='error',error=type(exc).__name__+': '+str(exc))
            progress(status['stage'])
            raise
        status.update(state='finished',error='')
        progress('complete')
        return summary


def compare_positions(baseline, assisted, output=None):
    """Two actual runs, common axes; never synthesize an unassisted flight."""
    import shutil
    sessions=[Path(baseline).resolve(),Path(assisted).resolve()]
    if sessions[0]==sessions[1]:raise ValueError('Select two separate experiments')
    traces=[];synthetic=False;modes=[]
    for session,expected in zip(sessions,[False,True]):
        manifest=load_json(session/'session.json')
        if manifest.get('state') not in ('recorded','recording_error'):
            raise ValueError('Stop both experiments before comparing')
        if manifest.get('flow_assistance_enabled') is not expected:
            raise ValueError('Expected assistance={} in {}'.format(expected,session))
        synthetic=synthetic or 'synthetic' in manifest.get('label','').lower()
        modes.append(manifest.get('flight_mode','waypoint'))
        rows=paper_csv_rows(session,'control_position.csv')
        reference=paper_csv_rows(session,'reference.csv')
        if not rows:raise ValueError('Missing actual controller position CSV in '+str(session))
        traces.append((rows,reference))
    if modes[0]!=modes[1]:raise ValueError('Use the same flight_mode in both position comparison experiments')
    out=Path(output).resolve() if output else sessions[1]/'position_comparison'
    out.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR',str(out/'.matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    def n(row,key):
        try:return float(row[key])
        except (KeyError,ValueError):return math.nan
    valid=lambda row: row.get('valid') in ('1','true','True')
    points=[(n(row,'x_m'),n(row,'y_m')) for rows,_ in traces for row in rows if valid(row)]
    points=[p for p in points if all(finite(x) for x in p)]
    if not points:raise ValueError('No valid controller positions to compare')
    bounds=[]
    for i in [0,1]:
        low=min(p[i] for p in points);high=max(p[i] for p in points);pad=max(.1,.1*(high-low));bounds.append((low-pad,high+pad))
    last_time=max(n(row,'t_s') for rows,_ in traces for row in rows)
    time_limits=(min(min(bounds[0]),min(bounds[1])),max(max(bounds[0]),max(bounds[1])))
    names=['uwb_only_position','flow_assisted_position']
    titles=['UWB-only position estimate used by controller','UWB with optical-flow fallback: controller estimate']
    for session,(rows,reference),name,title in zip(sessions,traces,names,titles):
        fig,axes=plt.subplots(1,2,figsize=(11,4.5))
        x=[n(r,'x_m') if valid(r) else math.nan for r in rows]
        y=[n(r,'y_m') if valid(r) else math.nan for r in rows]
        axes[0].plot(x,y,'.',markersize=1.6,label='Recorded controller estimate')
        if modes[0]!='waypoint':
            axes[0].plot([n(r,'goal_x_m') if r.get('trajectory_running')=='1' else math.nan for r in reference],
                         [n(r,'goal_y_m') if r.get('trajectory_running')=='1' else math.nan for r in reference],
                         'r--',label='Executed desired trajectory')
        targets=[]
        for r in reference:
            p=(n(r,'goal_x_m'),n(r,'goal_y_m'))
            if r.get('goal_active')=='1' and r.get('trajectory_running')!='1' and all(finite(v) for v in p) and (not targets or p!=targets[-1]):targets.append(p)
        if targets:axes[0].plot([p[0] for p in targets],[p[1] for p in targets],'rx',label='Targets')
        axes[0].set_xlim(*bounds[0]);axes[0].set_ylim(*bounds[1]);axes[0].set_aspect('equal',adjustable='box')
        axes[0].set_xlabel('map X [m]');axes[0].set_ylabel('map Y [m]');axes[0].legend()
        times=[n(r,'t_s') for r in rows]
        for key,label in [('x_m','X'),('y_m','Y')]:
            tx=[];values=[]
            for i,r in enumerate(rows):
                if i and (times[i]-times[i-1]>.5 or times[i]<times[i-1]):tx.append(math.nan);values.append(math.nan)
                tx.append(times[i]);values.append(n(r,key) if valid(r) else math.nan)
            axes[1].plot(tx,values,label=label)
        axes[1].set_xlim(0,last_time);axes[1].set_ylim(*time_limits);axes[1].set_ylabel('Position estimate [m]');axes[1].set_xlabel('Time since recording start [s]');axes[1].legend()
        for ax in axes:ax.grid(alpha=.25)
        fig.suptitle(('SYNTHETIC VALIDATION — ' if synthetic else '')+title);fig.tight_layout(rect=(0,0,1,.95))
        fig.savefig(out/(name+'.png'),dpi=300);fig.savefig(out/(name+'.pdf'));plt.close(fig)
        shutil.copyfile(session/'control_position.csv',out/(name+'.csv'))
    atomic_json(out/'comparison.json',{'baseline':str(sessions[0]),'assisted':str(sessions[1]),
        'plots':names,'synthetic_validation':synthetic,'notes':'Separate recorded experiments; common XY/ordinate limits and time axis, no offline filtering or resampling. These are position estimates, not independent physical ground truth. Match task, gains and occlusion protocol before interpreting differences.'})
    print('Two recorded position figures: '+str(out))
    return out


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
        status=json.loads(result[-1])
        if status.get('directory'):
            status['report_export']=load_json(Path(status['directory'])/'export_status.json',{'state':'not_started'})
        return status
    try:
        if args.command in ('start','stop'):
            client=node.create_client(Trigger,'/experiment/'+args.command)
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Experiment recorder unavailable; launch position_hold or experiment_logging first')
            future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=args.timeout)
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
    for command in ('start','stop','status'):
        action=sub.add_parser(command)
        if command in ('start','stop'):
            action.add_argument('--timeout',type=float,default=300.,help='Maximum service wait including automatic export [s]')
    mark=sub.add_parser('mark');mark.add_argument('text')
    export=sub.add_parser('export');export.add_argument('session',nargs='?',default='latest');export.add_argument('--root',default='log/experiments');export.add_argument('--recover',action='store_true',help='Export committed data after a crash; do not use while still recording')
    listing=sub.add_parser('list');listing.add_argument('--root',default='log/experiments')
    opening=sub.add_parser('open');opening.add_argument('session',nargs='?',default='latest');opening.add_argument('--root',default='log/experiments')
    compare=sub.add_parser('compare-positions');compare.add_argument('baseline');compare.add_argument('assisted');compare.add_argument('--output')
    args=parser.parse_args(argv)
    if hasattr(args,'timeout') and (not math.isfinite(args.timeout) or args.timeout<=0):
        parser.error('--timeout must be positive and finite')
    try:
        if args.command=='compare-positions':compare_positions(args.baseline,args.assisted,args.output)
        elif args.command=='export':export_session(resolve_session(args), args.recover)
        elif args.command=='open':
            session=resolve_session(args)
            state=load_json(session/'export_status.json').get('state')
            if state in ('exporting','error'):
                raise ValueError('Report export is '+state+'; inspect export_status.json or export again after it stops')
            path=session/'report.html'
            if not path.exists():raise ValueError('Report not available; stop recording or export first')
            subprocess.Popen(['xdg-open',str(path)])
        elif args.command=='list':
            root=Path(args.root)
            for path in sorted(root.glob('*/session.json')):
                m=load_json(path);report=load_json(path.parent/'export_status.json')
                report_state=report.get('state','finished' if (path.parent/'report.html').exists() else 'missing')
                print('{}  {}  {} messages  report={}'.format(path.parent,m.get('state'),m.get('written_messages',0),report_state))
        else:online(args)
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print('flight_log: '+str(exc),file=sys.stderr);return 1
    return 0


if __name__=='__main__':sys.exit(main())
