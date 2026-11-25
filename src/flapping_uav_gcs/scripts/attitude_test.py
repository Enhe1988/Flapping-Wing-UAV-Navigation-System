#!/usr/bin/env python3
"""Direct pitch/roll angle steps through the compact attitude bridge."""
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
from flapping_uav_msgs.msg import BodyAttitudeSetpoint


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--command',nargs=2,type=float,metavar=('PITCH_DEG','ROLL_DEG'))
    parser.add_argument('--duration',type=float,default=3.)
    parser.add_argument('--log-root',default='log/attitude_tests')
    args=parser.parse_args()
    if not math.isfinite(args.duration) or args.duration<=0:parser.error('duration must be positive and finite')
    if args.command and not all(math.isfinite(x) for x in args.command):parser.error('angles must be finite')
    rclpy.init(args=[]);node=Node('attitude_test_node')
    output=node.create_publisher(BodyAttitudeSetpoint,'/fc/attitude_setpoint_body',1)
    folder=Path(args.log_root);folder.mkdir(parents=True,exist_ok=True)
    path=folder/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_attitude_test.csv')
    stream=path.open('x',newline='',buffering=1);writer=csv.writer(stream)
    writer.writerow(['wall_time_utc','elapsed_s','event','pitch_deg','roll_deg','valid','reason'])
    origin=time.monotonic();angles=[0.,0.];deadline=None;stopping=False;conflict=False;exiting=False;single_started=False;done=None
    pending=None;last_sent_valid=None

    def record(event,pitch,roll,valid,reason=''):
        writer.writerow([datetime.now(timezone.utc).isoformat(timespec='milliseconds'),time.monotonic()-origin,event,pitch,roll,int(valid),reason])

    def publish():
        m=BodyAttitudeSetpoint();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='fc_attitude_target'
        m.valid=True;m.pitch_deg,m.roll_deg=angles;m.reason='Direct pitch/roll angle test, degrees'
        output.publish(m);record('publish',*angles,True,m.reason)

    def command(pitch,roll,duration):
        nonlocal deadline,pending,conflict
        if not exiting and node.count_publishers('/fc/attitude_setpoint_body')>1:
            conflict=True;raise ValueError('存在其他角度发布者，请先退出导航')
        if not all(math.isfinite(x) for x in (pitch,roll,duration)) or duration<=0:raise ValueError('角度须为有限值，时间须大于0')
        angles[:]=[max(-30.,min(30.,pitch)),max(-30.,min(30.,roll))]
        deadline=time.monotonic()+duration if any(angles) else None
        pending=time.monotonic();record('input',*angles,True)
        print('[{}] pitch={:+.2f}° roll={:+.2f}°，持续{}秒（每轴限幅±30°）'.format(
            datetime.now().isoformat(timespec='milliseconds'),*angles,duration),flush=True)
        publish()

    def tick():
        nonlocal conflict,done,exiting
        if node.count_publishers('/fc/attitude_setpoint_body')>1:
            conflict=True;return
        if not exiting and deadline is not None and time.monotonic()>=deadline:
            command(0.,0.,args.duration)
            if args.command is not None:done=time.monotonic()
        publish()

    def sent(m):
        nonlocal pending,last_sent_valid
        record('udp_sent',m.pitch_deg,m.roll_deg,m.valid,m.reason)
        if not m.valid and last_sent_valid is not False:
            print('当前发送禁用零角度：'+m.reason,flush=True)
        last_sent_valid=m.valid
        if pending is not None and m.valid and abs(m.pitch_deg-angles[0])<=.00501 and abs(m.roll_deg-angles[1])<=.00501:
            print('匹配UDP角度回报，输入后{:.1f}ms（仅电脑端），{}'.format((time.monotonic()-pending)*1000,m.reason),flush=True);pending=None

    node.create_subscription(BodyAttitudeSetpoint,'/fc/attitude_setpoint_sent',sent,50)
    node.create_timer(.02,tick)

    def stop(signum,frame):
        nonlocal stopping
        stopping=True

    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    print('输入：pitch_deg roll_deg [秒数]；前倾pitch为正，左滚roll为正；stop/0回水平，q退出。',flush=True)
    print('仅角度测试，不依赖UWB；桥接需收到yaw。CSV: '+str(path.resolve()),flush=True)
    buffer=b''
    try:
        while rclpy.ok() and not stopping and not conflict:
            rclpy.spin_once(node,timeout_sec=.01)
            if args.command is not None:
                if not single_started and output.get_subscription_count()>0:
                    command(*args.command,args.duration);single_started=True
                    if not any(angles):done=time.monotonic()
                if not single_started and time.monotonic()-origin>10:
                    raise RuntimeError('未发现角度通信订阅者，请核对horizontal_control_mode:=attitude')
                if done is not None and time.monotonic()-done>.35:break
            elif select.select([sys.stdin],[],[],0)[0]:
                chunk=os.read(sys.stdin.fileno(),4096)
                if not chunk:break
                buffer+=chunk
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1);fields=line.decode('utf-8',errors='replace').strip().split()
                    if not fields:continue
                    if fields[0] in ('q','quit','exit'):stopping=True;break
                    try:
                        if fields in (['stop'],['0']):command(0.,0.,args.duration)
                        elif len(fields) in (2,3):command(float(fields[0]),float(fields[1]),float(fields[2]) if len(fields)==3 else args.duration)
                        else:raise ValueError('格式为 pitch_deg roll_deg [秒数]')
                    except ValueError as error:print('输入未采用：'+str(error),flush=True)
    finally:
        if not conflict and node.count_publishers('/fc/attitude_setpoint_body')<=1 and rclpy.ok():
            exiting=True;command(0.,0.,args.duration);end=time.monotonic()+.35
            while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        if conflict:print('发现其他角度发布者，已停止；请先退出导航。',flush=True)
        stream.close();node.destroy_node();rclpy.shutdown()
        print('角度测试结束，CSV已保存：'+str(path.resolve()),flush=True)
    return 2 if conflict else 0


if __name__=='__main__':
    sys.exit(main())
