#!/usr/bin/env python3
"""Exercise the backup launch using private DDS and loopback UDP only."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def frame(message_id, payload, extra, sequence):
    header = bytes([len(payload), sequence % 256, 1, 1, message_id])
    crc = 65535
    for value in header + payload + bytes([extra]):
        tmp = value ^ (crc & 255)
        tmp ^= (tmp << 4) & 255
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 65535
    return b'\xfe' + header + payload + struct.pack('<H', crc)


def run(velocity_rate=None):
    os.environ['ROS_DOMAIN_ID'] = str(130 + os.getpid() % 15)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from flapping_uav_msgs.msg import AttitudeState, BodyMotionState, BodyVelocitySetpoint, MavlinkFrame, RcOverride

    rclpy.init()
    node = rclpy.create_node('backup_link_probe')
    samples = {}
    inbound = set()
    node.create_subscription(AttitudeState, '/fc/attitude', lambda m: samples.update(attitude=m), 10)
    node.create_subscription(BodyMotionState, '/fc/body_motion', lambda m: samples.update(motion=m), 10)
    node.create_subscription(MavlinkFrame, '/fc/mavlink_rx', lambda m: inbound.add(m.message_id), 100)
    commands = node.create_publisher(BodyVelocitySetpoint, '/fc/velocity_setpoint_body', 10)
    rc = node.create_publisher(RcOverride, '/rc/override', 10)
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1', 0))
    peer.setblocking(False)
    reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reserve.bind(('127.0.0.1', 0))
    rx_port = reserve.getsockname()[1]
    reserve.close()
    process = None
    packets = []
    sequence = 0
    origin = time.monotonic()
    try:
        with tempfile.TemporaryDirectory(prefix='uwb-backup-check-') as folder:
            with (Path(folder) / 'node.log').open('w+') as log:
                launch_args = [
                    'ros2', 'launch', 'flapping_uav_bringup', 'communication_backup.launch.py',
                    'velocity_protocol:=legacy',
                    'bind_ip:=127.0.0.1', 'remote_ip:=127.0.0.1',
                    'rx_port:=' + str(rx_port), 'tx_port:=' + str(peer.getsockname()[1]),
                ]
                if velocity_rate is not None:
                    launch_args.append('velocity_rate_hz:=' + str(velocity_rate))
                process = subprocess.Popen(launch_args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

                def spin(seconds, feed=True, command=True):
                    nonlocal sequence
                    end = time.monotonic() + seconds
                    next_feed = 0
                    while time.monotonic() < end:
                        if process.poll() is not None:
                            raise RuntimeError('Backup communication exited early:\n' +
                                               (Path(folder) / 'node.log').read_text())
                        now = time.monotonic()
                        if now >= next_feed:
                            if feed:
                                device_us = 10000000 + int((now - origin) * 1e6)
                                q = math.sqrt(.5)
                                messages = [
                                    (0, struct.pack('<IBBBBB', 0, 2, 3, 0, 3, 3), 50),
                                    (31, struct.pack('<I7f', device_us // 1000, q, 0., 0., q, .1, -.2, .3), 246),
                                    (250, struct.pack('<Q3f10s', device_us, .2, -.1, .7, b'FLOW_EST'), 49),
                                ]
                                for msg_id, payload, extra in messages:
                                    peer.sendto(frame(msg_id, payload, extra, sequence), ('127.0.0.1', rx_port))
                                    sequence += 1
                            if command:
                                m = BodyVelocitySetpoint()
                                m.header.stamp = node.get_clock().now().to_msg()
                                m.header.frame_id = 'base_link_heading_frd'
                                m.valid = True
                                m.velocity.x = .25
                                m.velocity.y = .15
                                commands.publish(m)
                            override = RcOverride()
                            override.valid = override.device_connected = True
                            override.channels = [1500] * 18
                            rc.publish(override)
                            next_feed = now + .02
                        rclpy.spin_once(node, timeout_sec=.003)
                        while True:
                            try:
                                data, address = peer.recvfrom(4096)
                            except BlockingIOError:
                                break
                            require(address[1] == rx_port, 'UDP TX did not use the RX socket')
                            require(data[0] == 253, 'Expected MAVLink2 output')
                            packets.append((int.from_bytes(data[7:10], 'little'), data[10:10 + data[1]]))

                # The bridge must start even before any flight-controller data arrives.
                spin(1.3, feed=False, command=False)
                before = sum(msg_id == 84 for msg_id, payload in packets)
                rate_start = time.monotonic()
                spin(2.0)
                measured_rate = (sum(msg_id == 84 for msg_id, payload in packets) - before) / (time.monotonic() - rate_start)
                expected_rate = 10.0 if velocity_rate is None else velocity_rate
                require(abs(measured_rate - expected_rate) <= max(2., expected_rate * .15),
                        'Wrong actual ID84 send rate: expected {} Hz, observed {} Hz'.format(expected_rate, measured_rate))
                require({0, 31, 250} <= inbound, 'Missing inbound MAVLink 0/31/250')
                require(samples.get('attitude') and samples['attitude'].valid, 'ID31 not decoded')
                require(samples.get('motion') and samples['motion'].velocity_valid, 'ID250 not decoded')
                require(abs(samples['motion'].forward_velocity_mps - .2) < 1e-6 and
                        abs(samples['motion'].right_velocity_mps + .1) < 1e-6 and
                        abs(samples['motion'].height_up_m - .7) < 1e-6, 'ID250 contract changed')
                require(abs(samples['attitude'].angular_velocity.x - .1) < 1e-6, 'ID31 body rates lost')
                require(rc.get_subscription_count() == 0, 'RC override subscriber exists')
                names = {name for name, namespace in node.get_node_names_and_namespaces()}
                require('joy_node' not in names and 'manual_control_node' not in names, 'RC nodes started')
                require({msg_id for msg_id, payload in packets} == {0, 84}, 'Unexpected outgoing message or missing heartbeat/velocity')
                velocities = [struct.unpack('<I11fHBBB', payload.ljust(53, b'\0'))
                              for msg_id, payload in packets if msg_id == 84]
                require(any(abs(v[4] - .25) < 1e-6 and abs(v[5] - .15) < 1e-6 for v in velocities),
                        'ID84 must preserve forward/right positive for the current FC')
                require(all(v[6] == 0 and v[12:] == (3559, 1, 1, 8) for v in velocities), 'ID84 XY frame/mask changed')
                packets.clear()
                spin(.5, command=False)
                tail = [payload for msg_id, payload in packets if msg_id == 84]
                require(tail and struct.unpack_from('<3f', tail[-1].ljust(53, b'\0'), 16) == (0., 0., 0.),
                        'Stale command did not produce zero velocity')
                require(all(msg_id != 70 for msg_id, payload in packets), 'RC forwarded after timeout')
                process.send_signal(signal.SIGINT)
                process.wait(timeout=15)
                require(process.returncode == 0, 'Backup launch shutdown failed')
                return {'status': 'passed', 'hardware_accessed': False,
                        'configured_velocity_rate_hz': expected_rate, 'measured_velocity_rate_hz': measured_rate,
                        'checked': ['starts without peer', 'RX 0/31/250', 'ID31 angular rates',
                                    'TX heartbeat and ID84', 'shared RX/TX socket', 'XY-only mask/frame and Y sign',
                                    'configured ID84 send rate', 'no joystick or mapper', 'RC input ignored', 'stale command outputs zero']}
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        peer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    results = [run(), run(20.0)]
    print(json.dumps(results, ensure_ascii=False, indent=2))
