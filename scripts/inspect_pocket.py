#!/usr/bin/env python3
"""Read-only /joy commissioning monitor. Never publishes flight commands."""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def run(args):
    # The private domain keeps this temporary joy driver out of the project domain.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy

    root = Path(__file__).resolve().parents[1]
    (root / 'log').mkdir(exist_ok=True)
    report_path = Path(args.output) if args.output else root / 'log/pocket-inspection.json'
    axes_min = []
    axes_max = []
    button_states = []
    latest = None
    baseline = None
    messages = 0
    last_rx = 0.0
    first_rx = 0.0
    last_print = 0.0
    rclpy.init(args=[])
    node = rclpy.create_node('pocket_inspection')

    def receive(msg):
        nonlocal latest, baseline, messages, last_rx, first_rx
        if not all(math.isfinite(value) for value in msg.axes):
            raise RuntimeError('收到非有限轴值，请检查输入设备')
        shape = (len(msg.axes), len(msg.buttons))
        if baseline is not None and shape != (len(baseline.axes), len(baseline.buttons)):
            raise RuntimeError('输入设备轴/按钮数量变化，请重新运行并重新核对')
        if baseline is None:
            baseline = msg
            axes_min.extend(msg.axes)
            axes_max.extend(msg.axes)
            button_states.extend({value} for value in msg.buttons)
            first_rx = time.monotonic()
        for index, value in enumerate(msg.axes):
            axes_min[index] = min(axes_min[index], value)
            axes_max[index] = max(axes_max[index], value)
        for index, value in enumerate(msg.buttons):
            button_states[index].add(value)
        messages += 1
        latest = msg
        last_rx = time.monotonic()

    subscription = node.create_subscription(Joy, '/joy', receive, qos_profile_sensor_data)
    process = None
    status = 'failed'
    error = None
    started = time.monotonic()
    log_path = root / 'log/pocket-inspection-driver.log'
    try:
        with log_path.open('w') as driver_log:
            joy_executable = str(Path(get_package_prefix('joy')) / 'lib/joy/joy_node')
            process = subprocess.Popen(
                [joy_executable, '--ros-args',
                 '-p', 'device_name:=' + args.device_name,
                 '-p', 'deadzone:=0.0', '-p', 'autorepeat_rate:=20.0'],
                stdout=driver_log, stderr=subprocess.STDOUT)
            print('只读核对：{}；ROS domain {}；不发送飞控消息。'.format(args.label, args.domain))
            print('一次只操作一个控件。轴/按钮均从 0 编号；us 是假设端点下的预览。', flush=True)
            while time.monotonic() - started < args.duration:
                if process.poll() is not None:
                    raise RuntimeError('joy_node 已退出，查看 ' + str(log_path))
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                if latest is None:
                    if now - started > 8:
                        raise RuntimeError('8 秒内未收到 /joy；查看 ' + str(log_path))
                    continue
                if now - last_rx > 1:
                    raise RuntimeError('/joy 已超过 1 秒未更新')
                if now - last_print >= 0.5:
                    fields = []
                    for index, value in enumerate(latest.axes):
                        preview = round(1500 + 500 * max(-1, min(1, value)))
                        fields.append('a{}={:+.3f}({}us)'.format(index, value, preview))
                    active = [i for i, value in enumerate(latest.buttons) if value]
                    print(' | '.join(fields) + ' | buttons=' + str(active), flush=True)
                    last_print = now
            if latest is None:
                raise RuntimeError('未收到 /joy')
            status = 'observed'
    except KeyboardInterrupt:
        status = 'interrupted'
    except Exception as exc:
        error = str(exc)
        print(error, file=sys.stderr)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report = {
            'recorded_at': datetime.now().astimezone().isoformat(),
            'label': args.label, 'status': status, 'error': error,
            'device_name': args.device_name, 'ros_domain_id': args.domain,
            'messages': messages, 'mapping_confirmed': False,
            'flight_commands_sent': False,
            'configured_deadzone': 0.0,
            'axis_observed_ranges': [
                {'index': i, 'min': low, 'max': high, 'span': high - low}
                for i, (low, high) in enumerate(zip(axes_min, axes_max))],
            'button_observed_states': [sorted(values) for values in button_states],
            'last_axes': list(latest.axes) if latest is not None else [],
            'last_buttons': list(latest.buttons) if latest is not None else [],
            'received_hz': ((messages - 1) / (last_rx - first_rx)
                            if messages > 1 and last_rx > first_rx else None),
            'driver_log': str(log_path),
        }
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print('记录：' + str(report_path))
    return 0 if status == 'observed' else 1


def main():
    parser = argparse.ArgumentParser(description='Pocket ROS2 /joy 只读核对工具')
    parser.add_argument('--duration', type=float, default=30, help='采集秒数，默认 30')
    parser.add_argument('--label', default='manual inspection', help='当前物理控件及档位')
    parser.add_argument('--output', help='本次记录 JSON 路径，默认覆盖 log/pocket-inspection.json')
    parser.add_argument('--domain', type=int, default=174, help='临时 ROS domain，默认 174')
    parser.add_argument('--device-name', default='EdgeTX Radiomaster Pocket Joystick')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration 必须是正数')
    if not 0 <= args.domain <= 232:
        parser.error('--domain 必须在 0..232')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
