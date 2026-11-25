#!/usr/bin/env python3
"""Validate the direct velocity console with private DDS and loopback UDP."""
import csv
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


def require(value, message):
    if not value:
        raise RuntimeError(message)


def main():
    os.environ['ROS_DOMAIN_ID'] = str(200 + os.getpid() % 10)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from flapping_uav_msgs.msg import BodyVelocitySetpoint
    rclpy.init()
    probe = rclpy.create_node('direct_velocity_probe')
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1', 0))
    peer.setblocking(False)
    reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reserve.bind(('127.0.0.1', 0))
    rx = reserve.getsockname()[1]
    reserve.close()
    processes = []
    packets = []
    try:
        with tempfile.TemporaryDirectory(prefix='uwb-direct-velocity-check-') as folder:
            root = Path(folder)
            with (root / 'bridge.log').open('w') as log:
                bridge = subprocess.Popen([
                    'ros2', 'launch', 'flapping_uav_bringup', 'communication_backup.launch.py',
                    'velocity_protocol:=legacy',
                    'bind_ip:=127.0.0.1', 'remote_ip:=127.0.0.1',
                    'rx_port:=' + str(rx), 'tx_port:=' + str(peer.getsockname()[1])],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(bridge)

            def spin(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    require(bridge.poll() is None, 'Bridge died: ' + (root / 'bridge.log').read_text())
                    rclpy.spin_once(probe, timeout_sec=.005)
                    while True:
                        try:
                            data, _ = peer.recvfrom(4096)
                        except BlockingIOError:
                            break
                        require(data[0] == 253, 'Expected MAVLink2')
                        mid = int.from_bytes(data[7:10], 'little')
                        payload = data[10:10 + data[1]].ljust(53, b'\0')
                        packets.append((time.monotonic(), mid, payload))

            def start(name, extra=()):
                with (root / (name + '.log')).open('w') as log:
                    process = subprocess.Popen([
                        'ros2', 'run', 'flapping_uav_gcs', 'velocity_test.py',
                        '--log-root', str(root / name), *extra],
                        stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
                        text=True, start_new_session=True)
                processes.append(process)
                return process

            def send(process, line):
                process.stdin.write(line + '\n')
                process.stdin.flush()

            def velocities(since):
                return [(stamp, *struct.unpack_from('<3f', payload, 16))
                        for stamp, mid, payload in packets if mid == 84 and stamp >= since]

            def has_velocity(since, vx, vy):
                return any(abs(x - vx) < 1e-5 and abs(y - vy) < 1e-5 and z == 0
                           for _, x, y, z in velocities(since))

            spin(1.5)
            console = start('interactive')
            spin(2.)
            require(console.poll() is None, 'Console failed to start')
            first_input = time.monotonic()
            send(console, '.3 0 .6')
            spin(.4)
            require(has_velocity(first_input, .3, 0), 'Positive X was not sent without UWB/attitude/flow')
            first_udp = next(t for t, x, y, z in velocities(first_input) if abs(x - .3) < 1e-5)
            require(first_udp - first_input < .3, 'Direct step delayed more than .3 seconds on loopback')
            spin(.6)
            require(has_velocity(first_input + .75, 0, 0), 'Timed step did not return to zero')
            second_input = time.monotonic()
            send(console, '0 -.25 2')
            spin(.35)
            require(has_velocity(second_input, 0, -.25), 'Negative Y sign changed')
            send(console, 'nan 0 2')
            send(console, '2 0 2')
            send(console, '0 0 -1')
            spin(.2)
            send(console, 'stop')
            stopped = time.monotonic()
            spin(.3)
            require(has_velocity(stopped, 0, 0), 'Manual stop failed')
            send(console, '.2 .1 5')
            spin(.3)
            exited = time.monotonic()
            # A terminal Ctrl+C reaches both ros2 run and its Python child.
            os.killpg(console.pid, signal.SIGINT)
            spin(.7)
            console.wait(timeout=3)
            require(console.returncode == 0, 'SIGINT shutdown failed: ' + (root / 'interactive.log').read_text())
            require(has_velocity(exited, 0, 0), 'SIGINT did not send zero')
            rows = list(csv.DictReader(next((root / 'interactive').glob('*.csv')).open()))
            require({'input', 'publish', 'udp_sent'} <= {r['event'] for r in rows}, 'CSV events missing')
            require(all(math.isfinite(float(r['vx_mps'])) for r in rows), 'Invalid command reached CSV output')
            require('输入未采用' in (root / 'interactive.log').read_text(), 'Bad commands were not rejected')
            one_input = time.monotonic()
            one = start('one_shot', ['--command', '-.2', '.1', '--duration', '.4'])
            spin(2.5)
            one.wait(timeout=3)
            require(one.returncode == 0, 'One-shot failed: ' + (root / 'one_shot.log').read_text())
            require(has_velocity(one_input, -.2, .1), 'One-shot command missing')
            require(velocities(one_input)[-1][1:] == (0., 0., 0.), 'One-shot left nonzero command')
            conflict = start('conflict')
            spin(1.5)
            other = probe.create_publisher(BodyVelocitySetpoint, '/fc/velocity_setpoint_body', 1)
            spin(1.5)
            conflict.wait(timeout=3)
            require(conflict.returncode != 0, 'Other publisher did not stop test')
            probe.destroy_publisher(other)
            require(all(mid != 70 for _, mid, _ in packets), 'Unexpected RC forwarding')
            require(any(mid == 0 for _, mid, _ in packets), 'Heartbeat missing')
            for _, mid, payload in packets:
                if mid == 84:
                    require(struct.unpack_from('<HBBB', payload, 48) == (3559, 1, 1, 8), 'ID84 contract changed')
            print(json.dumps({'status': 'passed', 'hardware_accessed': False,
                              'first_step_to_loopback_udp_ms': round((first_udp - first_input) * 1000, 2),
                              'checked': ['direct XY with no telemetry or localization', 'timed zero',
                                          'Y sign', 'input validation', 'manual stop', 'SIGINT zero',
                                          'CSV events', 'one-shot command', 'competing publisher exit',
                                          'heartbeat', 'no RC', 'ID84 XY frame and mask']}, indent=2))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        peer.close()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
