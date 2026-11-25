#!/usr/bin/env python3
"""Direct heading-frame XY velocity steps through the existing MAVLink bridge."""
import argparse
import csv
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import select
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from flapping_uav_msgs.msg import BodyMotionState, BodyVelocitySetpoint


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--command', nargs=2, type=float, metavar=('VX', 'VY'),
                        help='send one timed step and exit; otherwise read interactive commands')
    parser.add_argument('--duration', type=float, default=3., help='default step duration, seconds')
    parser.add_argument('--max-speed', type=float, default=1., help='XY vector norm limit, m/s')
    parser.add_argument('--rate', type=float, default=50., help='ROS publication rate; UDP stays at bridge rate')
    parser.add_argument('--log-root', default='log/velocity_tests')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.duration, args.max_speed, args.rate)):
        parser.error('duration, max-speed and rate must be finite and positive')
    if not 10 <= args.rate <= 100:
        parser.error('rate must be between 10 and 100 Hz')
    if args.command and (not all(math.isfinite(v) for v in args.command) or
                         math.hypot(*args.command) > args.max_speed):
        parser.error('command must be finite and within max-speed')
    return args


class VelocityTest(Node):
    def __init__(self, args):
        super().__init__('velocity_test_node')
        self.args = args
        self.origin = time.monotonic()
        self.velocity = (0., 0.)
        self.deadline = None
        self.change_time = None
        self.step = 0
        self.awaiting_echo = False
        self.last_sent_valid = None
        self.conflict = False
        self.single_started = False
        self.single_done = None
        self.exiting = False
        folder = Path(args.log_root)
        folder.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_velocity_test.csv'
        self.log_path = folder / name
        self.stream = self.log_path.open('x', newline='', buffering=1)
        self.csv = csv.writer(self.stream)
        self.csv.writerow(['wall_time_utc', 'elapsed_s', 'event', 'step',
                           'vx_mps', 'vy_mps', 'valid', 'since_input_ms', 'detail'])
        self.output = self.create_publisher(BodyVelocitySetpoint, '/fc/velocity_setpoint_body', 1)
        self.create_subscription(BodyVelocitySetpoint, '/fc/velocity_setpoint_sent', self.sent, 50)
        self.create_subscription(BodyMotionState, '/fc/body_motion', self.motion, 50)
        self.create_timer(1. / args.rate, self.tick)
        print('机体水平速度：X向前为正，Y向右为正，单位m/s；Z始终为0。', flush=True)
        print('输入：vx vy [持续秒数]；stop/0 回零；q 退出。默认持续{}秒。'.format(args.duration), flush=True)
        print('不使用定位、姿态或位置环，不做平滑/加速度限制；UDP频率由通信节点决定。', flush=True)
        print('CSV: {}'.format(self.log_path.resolve()), flush=True)

    def record(self, event, vx, vy, valid, detail='', elapsed_ms=None):
        now = time.monotonic()
        age = (now - self.change_time) * 1000 if self.change_time is not None else ''
        self.csv.writerow([datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                           '{:.6f}'.format(now - self.origin), event, self.step,
                           vx, vy, int(valid), age if elapsed_ms is None else elapsed_ms, detail])

    def set_command(self, vx, vy, duration, detail='manual step'):
        if not self.exiting and self.count_publishers('/fc/velocity_setpoint_body') > 1:
            self.conflict = True
            raise ValueError('存在其他速度发布者，测试停止；请先关闭导航/速度转换节点')
        if not all(math.isfinite(v) for v in (vx, vy, duration)) or duration <= 0:
            raise ValueError('速度须为有限数值，持续时间须大于0')
        if math.hypot(vx, vy) > self.args.max_speed + 1e-9:
            raise ValueError('速度矢量超过限制 {} m/s'.format(self.args.max_speed))
        self.step += 1
        self.velocity = (vx, vy)
        self.change_time = time.monotonic()
        self.deadline = self.change_time + duration if vx or vy else None
        self.awaiting_echo = True
        self.record('input', vx, vy, True, detail)
        print('[{} step={}] 输入 vx={:+.3f} vy={:+.3f}，{}秒后回零'.format(
            datetime.now().isoformat(timespec='milliseconds'), self.step, vx, vy, duration), flush=True)
        self.publish()

    def publish(self):
        message = BodyVelocitySetpoint()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link_heading_frd'
        message.valid = True
        # This input already IS body heading velocity; no attitude-based rotation
        # is performed or needed. Zero attitude age is deliberate in this test.
        message.command_age_ms = 0.
        message.attitude_age_ms = 0.
        message.reason = 'Direct body XY test; no localization or attitude rotation; step={}'.format(self.step)
        message.velocity.x, message.velocity.y = self.velocity
        self.output.publish(message)
        self.record('publish', *self.velocity, True, message.reason)

    def tick(self):
        if self.count_publishers('/fc/velocity_setpoint_body') > 1:
            if not self.conflict:
                print('检测到其他速度发布者，请停止导航/速度转换节点后重试；测试已停止发布。', flush=True)
            self.conflict = True
            return
        if self.conflict:
            return
        if self.exiting:
            self.publish()
            return
        if self.deadline is not None and time.monotonic() >= self.deadline:
            self.set_command(0., 0., self.args.duration, 'duration expired; zero')
            if self.args.command is not None:
                self.single_done = time.monotonic()
        if self.args.command is not None and not self.single_started:
            # Wait for discovery before timing a one-shot step.
            if self.output.get_subscription_count() == 0:
                if time.monotonic() - self.origin > 10:
                    print('10秒内未发现通信订阅者，测试退出。', flush=True)
                    self.conflict = True
                return
            self.single_started = True
            self.set_command(*self.args.command, self.args.duration)
            if not any(self.args.command):
                self.single_done = time.monotonic()
        self.publish()

    def sent(self, message):
        self.record('udp_sent', message.velocity.x, message.velocity.y, message.valid, message.reason)
        if not message.valid and self.last_sent_valid is not False:
            print('通信输出无效零速度：{}；请核对通信参数和速度上限。'.format(message.reason), flush=True)
        self.last_sent_valid = message.valid
        # Compact transport rounds to integer mm/s before transmission.
        tolerance = .0005001 if 'ID248' in message.reason else 1e-5
        matching = (abs(message.velocity.x - self.velocity[0]) < tolerance and
                    abs(message.velocity.y - self.velocity[1]) < tolerance)
        if self.awaiting_echo and message.valid and matching:
            self.awaiting_echo = False
            # Both stamps are on this PC. This measures PC publication through
            # the bridge monitor, NOT RF transport or flight-controller receipt.
            age_ms = (time.monotonic() - self.change_time) * 1000
            print('[step={}] 首个匹配UDP回报 vx={:+.3f} vy={:+.3f}，输入后{:.1f} ms（仅电脑端）'.format(
                self.step, message.velocity.x, message.velocity.y, age_ms), flush=True)

    def motion(self, message):
        self.record('fc_motion', message.forward_velocity_mps, message.right_velocity_mps,
                    message.velocity_valid, 'Measured ID250 velocity; not a command receipt ACK')

    def line(self, value):
        fields = value.strip().split()
        if not fields:
            return True
        if fields[0].lower() in ('q', 'quit', 'exit'):
            return False
        if fields in (['stop'], ['0']):
            self.set_command(0., 0., self.args.duration, 'manual zero')
            return True
        if len(fields) not in (2, 3):
            raise ValueError('请输入 vx vy [秒数]，或stop、q')
        numbers = [float(v) for v in fields]
        self.set_command(numbers[0], numbers[1], numbers[2] if len(numbers) == 3 else self.args.duration)
        return True

    def finish(self):
        # SIGINT is handled before rclpy shutdown so the final zero remains
        # publishable. The bridge also has its existing 200 ms input watchdog.
        if not self.conflict and rclpy.ok():
            self.exiting = True
            self.set_command(0., 0., self.args.duration, 'program exit; zero')
            until = time.monotonic() + .35
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=.01)
        self.stream.close()
        print('测试结束，CSV已保存：{}'.format(self.log_path.resolve()), flush=True)


def main():
    args = arguments()
    rclpy.init(args=[])
    stopping = False

    def stop_signal(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    node = VelocityTest(args)
    stdin_buffer = b''
    try:
        while rclpy.ok() and not stopping and not node.conflict:
            rclpy.spin_once(node, timeout_sec=.01)
            if args.command is not None:
                if node.single_done is not None and time.monotonic() - node.single_done > .35:
                    break
            elif select.select([sys.stdin], [], [], 0)[0]:
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    break
                stdin_buffer += chunk
                while b'\n' in stdin_buffer:
                    line, stdin_buffer = stdin_buffer.split(b'\n', 1)
                    try:
                        if not node.line(line.decode('utf-8', errors='replace')):
                            stopping = True
                            break
                    except ValueError as error:
                        print('输入未采用：{}'.format(error), flush=True)
    finally:
        node.finish()
        conflict = node.conflict
        node.destroy_node()
        rclpy.shutdown()
    return 2 if conflict else 0


if __name__ == '__main__':
    sys.exit(main())
