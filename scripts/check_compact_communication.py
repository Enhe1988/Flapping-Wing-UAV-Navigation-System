#!/usr/bin/env python3
"""Latest FC compact contract with loopback UDP, private ROS and synthetic UWB."""
import csv
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def frame(mid, payload, extra, sequence):
    header = bytes([len(payload), 0, 0, sequence % 256, 1, 1]) + mid.to_bytes(3, 'little')
    crc = 65535
    for value in header + payload + bytes([extra]):
        tmp = value ^ (crc & 255)
        tmp ^= (tmp << 4) & 255
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 65535
    return b'\xfd' + header + payload + struct.pack('<H', crc)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--attitude',action='store_true',help='exercise direct pitch/roll control (type 0x8002)')
    angle_mode=parser.parse_args().attitude
    mode='attitude' if angle_mode else 'velocity'
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('FLAPPING_COMPACT_TEST_DOMAIN',str(215 + os.getpid() % 10))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from flapping_uav_msgs.msg import AttitudeState, BodyVelocitySetpoint, BodyAttitudeSetpoint, NavigationStatus, UwbStatus
    from flapping_uav_msgs.msg import PositionAttitudeDebug
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node('compact_contract_probe')
    last = {}
    node.create_subscription(AttitudeState, '/fc/attitude', lambda m: last.update(attitude=m), 10)
    node.create_subscription(NavigationStatus, '/navigation/status', lambda m: last.update(nav=m), 10)
    if angle_mode:
        node.create_subscription(PositionAttitudeDebug, '/navigation/attitude_pid', lambda m: last.update(pid=m), 10)
    node.create_subscription(BodyAttitudeSetpoint if angle_mode else BodyVelocitySetpoint,
                             '/fc/attitude_setpoint_body' if angle_mode else '/fc/velocity_setpoint_body',
                             lambda m: last.update(command=m), 10)
    pose = node.create_publisher(PoseWithCovarianceStamped, '/uwb/position', 10)
    uwb = node.create_publisher(UwbStatus, '/uwb/status', 10)
    goal = node.create_publisher(PoseStamped, '/navigation/goal', 10)
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1', 0)); peer.setblocking(False)
    reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reserve.bind(('127.0.0.1', 0)); rx = reserve.getsockname()[1]; reserve.close()
    folder = ROOT / 'log/experiments_demo' / (('attitude_validation_' if angle_mode else 'compact_validation_') + str(time.monotonic_ns()))
    folder.mkdir(parents=True)
    processes = []
    packets = []
    origin = time.monotonic()
    next_yaw = next_heartbeat = next_uwb = 0.
    sequence = 0
    yaw = .4
    boot_base = 10000
    feed_yaw = feed_uwb = True
    pose_xy = [2., -3.]
    injected=None;direct_angle_publisher=None

    def launch(name, arguments, stdin=False):
        with (folder / (name + '.log')).open('w') as log:
            process = subprocess.Popen(arguments, stdin=subprocess.PIPE if stdin else None,
                                       stdout=log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        processes.append(process)
        return process

    def advance(seconds):
        nonlocal next_yaw, next_heartbeat, next_uwb, sequence
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = time.monotonic(); elapsed = now - origin
            if now >= next_yaw:
                if feed_yaw:
                    tick = boot_base + int(elapsed * 1000)
                    peer.sendto(frame(254, struct.pack('<IfB', tick, yaw, 1), 46, sequence), ('127.0.0.1', rx))
                    sequence += 1
                next_yaw = now + .2  # Actual firmware: 5Hz, not the initial proposal's 20Hz.
            if now >= next_heartbeat:
                peer.sendto(frame(0, struct.pack('<IBBBBB', 0, 17, 0, 0, 0, 3), 50, sequence), ('127.0.0.1', rx))
                sequence += 1; next_heartbeat = now + 1
            if now >= next_uwb:
                if feed_uwb:
                    p = PoseWithCovarianceStamped(); p.header.stamp = node.get_clock().now().to_msg(); p.header.frame_id = 'map'
                    p.pose.pose.position.x, p.pose.pose.position.y = pose_xy; pose.publish(p)
                    u = UwbStatus(); u.header = p.header; u.position_valid = u.coordinates_confirmed = True; uwb.publish(u)
                if injected is not None:direct_angle_publisher.publish(injected)
                next_uwb = now + .02
            rclpy.spin_once(node, timeout_sec=.003)
            while True:
                try:
                    data, _ = peer.recvfrom(4096)
                except BlockingIOError:
                    break
                require(data[0] == 253, 'Compact transport must be MAVLink2')
                mid = int.from_bytes(data[7:10], 'little'); payload = data[10:10 + data[1]]
                require(mid in (0, 248), 'Unexpected outgoing ID (RC/84 must not be sent): ' + str(mid))
                if mid == 248:
                    require(6 <= len(payload) <= 15, 'Firmware rejects this ID248 length')
                    full = payload.ljust(15, b'\0')
                    require(struct.unpack_from('<HBBB', full) == (0x8002 if angle_mode else 0x8001, 0, 1, 1), 'Wrong extension type/target')
                    version, enabled, vx, vy, echo = struct.unpack_from('<BBhhI', full, 5)
                    require(version == 1 and enabled in (0, 1), 'Wrong compact payload version/enable')
                    packets.append((now, enabled, vx, vy, echo))

    def call(name):
        client = node.create_client(Trigger, name)
        try:
            deadline = time.monotonic() + 8
            while not client.service_is_ready() and time.monotonic() < deadline:
                advance(.05)
            require(client.service_is_ready(), 'Missing service ' + name)
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 5
            while not future.done() and time.monotonic() < deadline:
                advance(.02)
            require(future.done(), 'Service timeout ' + name)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        bridge = launch('bridge', ['ros2', 'launch', 'flapping_uav_bringup', 'communication_backup.launch.py',
                        'horizontal_control_mode:='+mode,
                        'bind_ip:=127.0.0.1', 'remote_ip:=127.0.0.1', 'rx_port:=' + str(rx),
                        'tx_port:=' + str(peer.getsockname()[1])])
        advance(1.5)
        require(last.get('attitude') and last['attitude'].yaw_only, 'No ID254 heading decoded')
        require(math.isnan(last['attitude'].orientation.w), 'Missing roll/pitch became a fake full quaternion')
        console = launch('console', ['ros2', 'run', 'flapping_uav_gcs', 'attitude_test.py' if angle_mode else 'velocity_test.py',
                                    '--log-root', str(folder / 'console_csv')], stdin=True)
        advance(1.5)
        before = time.monotonic(); console.stdin.write('6.78 -12.34 .6\n' if angle_mode else '.321 -.456 .6\n'); console.stdin.flush()
        advance(.4)
        expected=(1,678,-1234) if angle_mode else (1,321,-456)
        matches = [p for p in packets if p[0] >= before and p[1:4] == expected]
        require(matches and matches[0][0] - before < .3, 'Direct console did not promptly send compact XY')
        require(boot_base <= matches[0][4] <= boot_base + int((time.monotonic() - origin) * 1000), 'Bad yaw echo')
        advance(.5)
        require(packets[-1][2:4] == (0, 0), 'Timed direct command did not zero')
        console.stdin.write('q\n'); console.stdin.flush(); advance(.7); console.wait(timeout=3)
        require(console.returncode == 0, 'Direct console failed: ' + (folder / 'console.log').read_text())
        if angle_mode:
            direct_angle_publisher=node.create_publisher(BodyAttitudeSetpoint,'/fc/attitude_setpoint_body',1)
            injected=BodyAttitudeSetpoint();injected.header.frame_id='fc_attitude_target';injected.valid=True
            injected.pitch_deg=60.;injected.roll_deg=-100.
            advance(.5)
            require(packets[-1][1:4]==(1,3000,-3000),'Bridge independent +/-30 degree clamp failed')
            injected=None;node.destroy_publisher(direct_angle_publisher);advance(.4)
            require(packets[-1][1:4]==(0,0,0),'Stale attitude input did not disable/zero')
        logging_config=folder/'alignment_logging.yaml'
        logging_config.write_text('experiment_recorder_node:\n  ros__parameters:\n    start_on_trajectory_tracking: false\n    export_on_stop: true\n')
        nav = launch('navigation', ['ros2', 'launch', 'flapping_uav_bringup', 'position_hold.launch.py',
                     'horizontal_control_mode:='+mode,
                     'start_uwb:=false', 'use_rviz:=false', 'flight_mode:=line', 'logging_config:='+str(logging_config),
                     'log_root:=' + str(folder / 'sessions'), 'experiment_label:=synthetic_compact'])
        advance(2.)
        response = call('/navigation/start_here')
        require(response.success, 'Yaw-only start_here failed: ' + response.message)
        advance(.7)
        require(last['command'].valid and last['nav'].goal_active, 'No hold without ID250')
        require(math.isnan(last['nav'].actual_speed_mps), 'Missing speed was reported as measured zero')
        require(not last['nav'].feedback_assumed_zero and not last['nav'].optical_flow_fallback_enabled, 'Wrong compact control mode')
        target = PoseStamped(); target.header.frame_id = 'map'; target.pose.position.x = 2.2; target.pose.position.y = -3.1
        goal.publish(target); advance(.7)
        require(any(p[1] and p[2] > 0 and (p[3]<0 if angle_mode else p[3]>0) for p in packets[-5:]), 'Aligned +X / -Y body output directions wrong')
        require(last['nav'].horizontal_control_mode==mode,'Wrong output mode')
        if angle_mode:
            require(math.isnan(last['nav'].velocity_map.x),'Angles mislabeled as m/s')
            require(last['pid'].valid and last['pid'].i_map_deg.x>0, 'Integral not active in near-target hold')
            pose_xy[0] += .04; advance(.2)
            require(last['pid'].velocity_valid and last['pid'].d_map_deg.x<0, 'UWB motion did not produce braking D')
            advance(.5)
            target.pose.position.x=100.;target.pose.position.y=-100.;goal.publish(target);advance(.85)
            require(packets[-1][2:4]==(3000,-3000),'Position-loop +/-30deg cap failed')
        response = call('/navigation/start_trajectory')
        require(response.success, 'Trajectory still gated by absent ID250: ' + response.message)
        advance(.3)
        require(last['nav'].trajectory_running, 'Trajectory not running')
        require(call('/navigation/stop_trajectory').success, 'Trajectory stop failed')
        feed_uwb = False; advance(.75)
        require(not last['command'].valid and last['nav'].goal_active, 'Short UWB loss did not pause and retain goal')
        if angle_mode:
            require(not last['pid'].valid and last['pid'].i_map_deg.x==0 and last['pid'].i_map_deg.y==0, 'Input fault did not clear integral')
        feed_uwb = True; advance(.85)
        require(last['command'].valid and last['nav'].goal_active, 'UWB recovery failed')
        feed_uwb = False; advance(2.8)
        require(not last['nav'].goal_active and not last['command'].valid, 'Persistent UWB loss did not clear goal')
        feed_uwb = True; advance(.9)
        require(call('/navigation/start_here').success, 'Could not re-align')
        advance(.3); boot_base = 100; advance(.85)
        require(not last['nav'].goal_active and not last['command'].valid, 'FC reboot did not expire heading and goal')
        os.killpg(nav.pid, signal.SIGINT); nav.wait(timeout=40)
        require(nav.returncode == 0, 'Navigation/log shutdown failed: ' + (folder / 'navigation.log').read_text())
        sessions = list((folder / 'sessions').glob('*/attitude.csv'))
        require(sessions, 'No automatic yaw log')
        attitude_rows = list(csv.DictReader(sessions[0].open()))
        require(attitude_rows and all(r['yaw_only'] == '1' for r in attitude_rows), 'Yaw mode not recorded')
        require(all(math.isnan(float(r['roll_deg'])) and math.isnan(float(r['p_rad_s'])) for r in attitude_rows), 'Missing attitude data fabricated')
        require(any(math.isfinite(float(r['yaw_fc_deg'])) for r in attitude_rows), 'Yaw not logged')
        actual = list(csv.DictReader((sessions[0].parent / 'velocity_actual.csv').open()))
        require(not actual, 'Actual velocity fabricated in compact mode')
        chart='attitude_targets.png' if angle_mode else 'velocity.png'
        require((sessions[0].parent / ('report/'+chart)).is_file(), 'Automatic report missing')
        if angle_mode:
            angle_rows=list(csv.DictReader((sessions[0].parent/'attitude_sent.csv').open()))
            require(angle_rows and all(abs(float(r['pitch_deg']))<=30 and abs(float(r['roll_deg']))<=30 for r in angle_rows),'Angle CSV units/limits wrong')
            require(not list(csv.DictReader((sessions[0].parent/'velocity_desired.csv').open())),'Angle commands leaked into velocity CSV')
            pid_rows=list(csv.DictReader((sessions[0].parent/'attitude_pid.csv').open()))
            require(pid_rows and any(r['velocity_estimate_valid']=='1' for r in pid_rows),'UWB PID diagnostics missing')
            require(all(math.hypot(float(r['i_map_x_deg']),float(r['i_map_y_deg']))<=2+1e-9 for r in pid_rows),'Integral bound exceeded')
            require(all(math.hypot(float(r['d_map_x_deg']),float(r['d_map_y_deg']))<=8+1e-9 for r in pid_rows),'Derivative bound exceeded')
            require((sessions[0].parent/'report/attitude_pid.png').is_file(),'Automatic PID component chart missing')
        result = {'status': 'passed', 'hardware_accessed': False, 'firmware_commit': '74d6a5cb6cd8beaa4970d135d74738a4cea5a176',
                  'data_kind': 'SYNTHETIC COMPACT LINK AND UWB; NOT FLIGHT EVIDENCE',
                  'horizontal_control_mode':mode,
                  'log_root': str(folder), 'checked': ['actual 5Hz yaw contract', 'ID248 type 0x8001 version 1',
                  'mm/s signed XY and exact yaw clock echo', 'direct console timed zero', 'no ID70/84',
                  'start_here without ID250', 'UWB pure-P hold and waypoint', 'trajectory start/stop without flow',
                  'transient recovery', 'persistent loss clears goal', 'reboot clears heading', 'NaN missing telemetry',
                  'automatic yaw CSV and charts']}
        if angle_mode:
            result['firmware_commit']='dfe1d83a748459e77c12378f6a98e1499daaf55b'
            result['checked']=['type 0x8002 version 1 and centidegrees' if x=='ID248 type 0x8001 version 1' else x for x in result['checked']]
            result['checked']=['signed pitch/roll centidegrees and exact yaw clock echo' if x=='mm/s signed XY and exact yaw clock echo' else x for x in result['checked']]
            result['checked']=['UWB PID hold and waypoint' if x=='UWB pure-P hold and waypoint' else x for x in result['checked']]
            result['checked']+=['independent bridge +/-30deg clamp','stale command zero','position-loop pitch/roll signs and saturation','angle CSV and chart; no fabricated m/s',
                                'near-goal integral, UWB-derived braking and fault reset','bounded I/D components and automatic PID CSV/chart']
        (folder / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5)
        peer.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
