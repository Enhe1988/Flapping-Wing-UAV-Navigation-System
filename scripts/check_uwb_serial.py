#!/usr/bin/env python3
"""Feed synthetic Anchor_Frame0 bytes through a private PTY and the official driver.

Test orchestration only. The serial device is allocated by this process; no
physical serial device is opened. The fixture layout comes from the pinned
nlink_unpack/nlink_linktrack_anchorframe0.c (896 bytes, 24-bit millimetres).
"""

import argparse
import json
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import time
import tty


def packet(index):
    frame = bytearray(896)
    frame[0:2] = b'\x55\x00'
    for slot in range(30):
        frame[2 + slot * 27] = 255
    for slot, tag_id, xyz in [(0, 1, (99, 99, 99)), (1, 0, (1, 2, .8))]:
        offset = 2 + slot * 27
        # Match the Tag role verified on physical hardware (role=2).
        frame[offset:offset + 2] = bytes([tag_id, 2])
        for axis, value in enumerate(xyz):
            start = offset + 2 + axis * 3
            frame[start:start + 3] = int(value * 1000).to_bytes(3, 'little', signed=True)
    struct.pack_into('<I', frame, 879, index * 50)
    struct.pack_into('<H', frame, 887, 5000)
    struct.pack_into('<I', frame, 889, index * 50)
    frame[893:896] = b'\x07\x01\xee'
    return frame


def run(launch_file='uwb.launch.py'):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nlink_parser2.msg import LinktrackAnchorframe0
    from std_srvs.srv import Trigger

    root = Path(__file__).resolve().parents[1]
    log_dir = root / 'log'
    log_dir.mkdir(exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(80 + os.getpid() % 100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    master, slave = pty.openpty()
    tty.setraw(slave)
    device = os.ttyname(slave)
    process = probe = None
    raw_messages, positions = [], []
    hold = launch_file == 'position_hold.launch.py'
    log_path = log_dir / ('position-hold-serial.log' if hold else 'uwb-serial.log')
    try:
        rclpy.init()
        probe = Node('uwb_private_serial_probe')
        probe.create_subscription(LinktrackAnchorframe0, '/nlink_linktrack_anchorframe0',
                                  lambda msg: raw_messages.append(msg), 10)
        probe.create_subscription(PoseWithCovarianceStamped, '/uwb/position',
                                  lambda msg: positions.append((time.monotonic(), msg)), 10)
        with log_path.open('w') as log:
            ros2 = '/opt/ros/{}/bin/ros2'.format(os.environ['ROS_DISTRO'])
            process = subprocess.Popen(
                [sys.executable, ros2, 'launch', 'flapping_uav_bringup', launch_file,
                 'port_name:=' + device, 'baud_rate:=115200',
                 'coordinates_confirmed:=true', 'use_rviz:=false'] + (['record_experiment:=false'] if hold else []),
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            end = time.monotonic() + 8
            while 'Serial port opened successfully' not in log_path.read_text():
                if process.poll() is not None or time.monotonic() >= end:
                    raise RuntimeError(
                        'Official driver did not open the private PTY: ' + str(log_path))
                rclpy.spin_once(probe, timeout_sec=.02)
            started = time.monotonic()
            for index in range(120):
                data = packet(index)
                # Split writes; the OS may coalesce individual reads.
                os.write(master, data[:13])
                os.write(master, data[13:])
                deadline = started + (index + 1) * .05
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError('Hardware-mode launch exited during virtual input')
                    rclpy.spin_once(
                        probe, timeout_sec=min(.01, max(0, deadline - time.monotonic())))
            if len(positions) < 90 or len(raw_messages) < 90:
                raise RuntimeError('Too few messages decoded from the private serial stream')
            for _, message in positions:
                p = message.pose.pose.position
                if abs(p.x - 1) > 1e-5 or abs(p.y + 2) > 1e-5 or abs(p.z - .8) > 1e-5:
                    raise RuntimeError('Tag selection, raw byte scaling or Z preservation failed')
            if raw_messages[-1].nodes[0].id != 1 or raw_messages[-1].nodes[1].id != 0:
                raise RuntimeError('Raw Anchor fixture did not traverse the official parser')
            frequency = (len(positions) - 1) / (positions[-1][0] - positions[0][0])
            if not 18 <= frequency <= 22:
                raise RuntimeError('Decoded position frequency was not near 20 Hz')
            if hold:
                expected = {'position_controller_node', 'linktrack_parser',
                            'uwb_position_node', 'scene_visualizer_node'}
                if not expected.issubset(set(probe.get_node_names())):
                    raise RuntimeError('Position hold did not start the complete UWB/scene graph')
                start = probe.create_client(Trigger, '/navigation/start_here')
                if not start.wait_for_service(timeout_sec=2):
                    raise RuntimeError('Position hold start_here service missing')
                future = start.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(probe, future, timeout_sec=2)
                if not future.done() or future.result().success or not any(
                        reason in future.result().message for reason in
                        ('Heartbeat missing/stale', 'ID31 attitude missing')):
                    raise RuntimeError('Position hold must reject starting without FC telemetry')
            process.send_signal(signal.SIGINT)
            process.wait(timeout=8)
            if process.returncode != 0 or 'process has died' in log_path.read_text():
                raise RuntimeError('Official driver or C++ pipeline did not exit normally')
        return {'status': 'passed', 'hardware_tested': False,
                'launch_file': launch_file,
                'position_hold_graph_verified': hold,
                'test_kind': 'private PTY -> unchanged official C++ parser -> C++ UWB node -> DDS',
                'fixture_protocol': 'Anchor_Frame0', 'fixture_tag_id': 0,
                'fixture_baud_rate': 115200, 'physical_baud_rate_confirmed': False,
                'frames_written': 120, 'raw_messages_received': len(raw_messages),
                'positions_received': len(positions), 'received_hz': frequency,
                'tag0_not_first': True, 'millimetres_to_metres': 'verified',
                'native_uwb_y_reflection': 'verified: raw (1,2,.8) -> map (1,-2,.8)',
                'shutdown': 'official driver and project node exited normally'}
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(master)
        os.close(slave)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', choices=['uwb.launch.py', 'position_hold.launch.py'],
                        default='uwb.launch.py')
    args = parser.parse_args()
    filename = ('position-hold-serial-result.json' if args.launch == 'position_hold.launch.py'
                else 'uwb-serial-result.json')
    output = Path(__file__).resolve().parents[1] / 'log' / filename
    output.parent.mkdir(exist_ok=True)
    output.unlink(missing_ok=True)
    try:
        result = run(args.launch)
    except (RuntimeError, OSError, ImportError) as error:
        result = {'status': 'failed', 'error': str(error)}
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(output.read_text())
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
