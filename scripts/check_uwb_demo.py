#!/usr/bin/env python3
"""Test orchestration only: exercise the installed C++ stage 1 launch over DDS."""

from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def run():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Path as RosPath
    from flapping_uav_msgs.msg import UwbStatus
    from std_srvs.srv import Trigger

    root = Path(__file__).resolve().parents[1]
    log_dir = root / 'log'
    log_dir.mkdir(exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(80 + os.getpid() % 100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    ros2 = '/opt/ros/{}/bin/ros2'.format(os.environ['ROS_DISTRO'])
    process = probe = None
    positions = deque(maxlen=500)
    statuses = deque(maxlen=200)
    paths = deque(maxlen=5)
    results = {'status': 'passed', 'test_kind': 'C++ UWB launch and simulated DDS only',
               'protocol': 'Anchor_Frame0', 'tag_id': 0, 'hardware_tested': False}
    node_log = log_dir / 'uwb-demo.log'
    try:
        rclpy.init()
        probe = Node('uwb_demo_probe')
        probe.create_subscription(PoseWithCovarianceStamped, '/uwb/position',
                                  lambda msg: positions.append((time.monotonic(), msg)), 10)
        probe.create_subscription(UwbStatus, '/uwb/status',
                                  lambda msg: statuses.append(msg), 10)
        probe.create_subscription(RosPath, '/uwb/path', lambda msg: paths.append(msg), 10)

        def spin_for(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                if process is not None:
                    require(process.poll() is None, 'Launch exited; inspect ' + str(node_log))
                rclpy.spin_once(probe, timeout_sec=min(.01, max(0, end - time.monotonic())))

        def wait_until(predicate, timeout, error):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                spin_for(.02)
            require(predicate(), error)

        def stop_launch():
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)

        def start_launch(log, scenario):
            return subprocess.Popen(
                [sys.executable, ros2, 'launch', 'flapping_uav_bringup',
                 'uwb_demo.launch.py', 'use_rviz:=false', 'scenario:=' + scenario],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        with node_log.open('w') as log:
            process = start_launch(log, 'normal')
            wait_until(lambda: bool(positions) and bool(statuses), 8, 'No UWB DDS data received')
            positions.clear()
            spin_for(10.0)
            samples = list(positions)
            require(len(samples) >= 180, 'Fewer than 180 positions during 10 seconds')
            frequency = (len(samples) - 1) / (samples[-1][0] - samples[0][0])
            require(18 <= frequency <= 22, 'Position topic is not near 20 Hz')
            for _, msg in samples:
                require(msg.header.frame_id == 'map', 'Incorrect frame')
                require(abs(msg.pose.pose.position.x - 1) <= .02
                        and abs(msg.pose.pose.position.y - 2) <= .02,
                        'Wrong Tag selected or units changed')
                require(abs(msg.pose.pose.position.z - .8) < 1e-6, 'Observed Z was not preserved')
                require(msg.header.stamp.sec > 0, 'Ground ROS reception stamp is missing')
            latest = statuses[-1]
            require(latest.tag_id == 0 and latest.position_valid, 'Tag 0 not valid')
            require(latest.covariance_valid and latest.static_noise_samples == 100,
                    'Explicit static-noise commissioning did not finish')
            require(math.isfinite(latest.stddev_x_m) and latest.stddev_x_m > 0,
                    'Static noise estimate is invalid')
            require(paths and len(paths[-1].poses) <= 300,
                    'Path buffer exceeds its configured limit')
            require(all(p.pose.position.z == 0 for p in paths[-1].poses),
                    'Visualization is not the documented XY projection')
            results['normal'] = {'received_messages': len(samples), 'received_hz': frequency,
                                 'static_samples': latest.static_noise_samples,
                                 'stddev_x_m': latest.stddev_x_m,
                                 'stddev_y_m': latest.stddev_y_m,
                                 'path_points': len(paths[-1].poses)}
            client = probe.create_client(Trigger, '/uwb/reset_observation')
            require(client.wait_for_service(timeout_sec=3), 'Reset service not available')
            future = client.call_async(Trigger.Request())
            wait_until(future.done, 3, 'Reset service timed out')
            require(future.result().success, 'Reset service failed')
            statuses.clear()
            wait_until(lambda: statuses and statuses[-1].position_valid
                       and statuses[-1].static_noise_samples < 100, 2,
                       'Noise collection did not restart after reset')
            probe.destroy_client(client)
            results['reset_service'] = 'reset and resumed observations'
            stop_launch()
            require(process.returncode == 0, 'Normal launch failed to stop cleanly')
            require('process has died' not in node_log.read_text(), 'A C++ node exited abnormally')

        # A fresh launch exercises a genuine publisher pause and recovery.
        with (log_dir / 'uwb-dropout.log').open('w') as log:
            process = None
            spin_for(.2)
            positions.clear()
            statuses.clear()
            paths.clear()
            process = start_launch(log, 'dropout')
            wait_until(lambda: statuses and statuses[-1].position_valid, 8,
                       'Dropout demo did not begin with valid data')
            wait_until(lambda: statuses and statuses[-1].state == UwbStatus.STREAM_TIMEOUT, 6,
                       'Publisher pause was not detected')
            require(not statuses[-1].position_valid, 'Stale position remained valid')
            count_after_timeout = len(positions)
            spin_for(.1)
            require(len(positions) == count_after_timeout, 'Old positions were republished')
            wait_until(lambda: statuses and statuses[-1].position_valid, 4,
                       'Positions did not recover when the publisher resumed')
            results['dropout'] = (
                'timeout invalidated status, no old pose republish, automatic recovery')
            stop_launch()
            require(process.returncode == 0, 'Dropout launch failed to stop cleanly')
        results['shutdown'] = 'C++ nodes and launch exited normally'
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
    return results


def main():
    output = Path(__file__).resolve().parents[1] / 'log' / 'uwb-demo-result.json'
    output.parent.mkdir(exist_ok=True)
    output.unlink(missing_ok=True)
    try:
        result = run()
    except (RuntimeError, OSError, ImportError) as error:
        result = {'status': 'failed', 'error': str(error)}
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(output.read_text())
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
