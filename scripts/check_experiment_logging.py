#!/usr/bin/env python3
"""Private DDS synthetic flight -> C++ rosbag recorder -> offline report regression.
No control/communication nodes or physical devices are started.
"""
import csv
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time


def check(condition, message):
    if not condition:
        raise RuntimeError(message)


def run():
    os.environ['ROS_DOMAIN_ID'] = str(201 + os.getpid() % 20)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from sensor_msgs.msg import Joy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rcl_interfaces.msg import Log
    from flapping_uav_msgs.msg import AttitudeState, BodyMotionState, BodyVelocitySetpoint, HeadingAlignment, MavlinkFrame, NavigationStatus, RcOverride, UwbStatus
    root=Path(__file__).resolve().parents[1]
    output_root=root/'log'/'experiments_demo'
    log_path=root/'log'/'experiment-logging-validation.log'
    output_root.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('position_controller_node')
    node.declare_parameter('kp_xy', .42)
    node.declare_parameter('max_speed_mps', .25)
    node.declare_parameter('arrival_position_tolerance_m', .05)
    node.declare_parameter('quaternion_body_to_ned', True)
    live=QoSProfile(depth=100)
    retained=QoSProfile(depth=1,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    specs={
        'pose':(PoseWithCovarianceStamped,'/uwb/position'), 'uwb':(UwbStatus,'/uwb/status'),
        'att':(AttitudeState,'/fc/attitude'), 'motion':(BodyMotionState,'/fc/body_motion'),
        'body':(BodyVelocitySetpoint,'/fc/velocity_setpoint_body'), 'sent':(BodyVelocitySetpoint,'/fc/velocity_setpoint_sent'),
        'nav':(NavigationStatus,'/navigation/status'), 'rc':(RcOverride,'/rc/override'),
        'joy':(Joy,'/joy'), 'diag':(DiagnosticArray,'/diagnostics'), 'raw':(MavlinkFrame,'/fc/mavlink_rx'),
        'map':(TwistStamped,'/navigation/velocity_setpoint_map'), 'note':(String,'/experiment/mark'),
        'rosout':(Log,'/rosout'),
    }
    publishers={key:node.create_publisher(typ,topic,retained if key=='rosout' else live) for key,(typ,topic) in specs.items()}
    alignment=node.create_publisher(HeadingAlignment,'/navigation/heading_alignment',retained)
    process=None
    stream=log_path.open('w')
    try:
        process=subprocess.Popen(['ros2','launch','flapping_uav_bringup','experiment_logging.launch.py',
                                  'log_root:='+str(output_root),'experiment_label:=synthetic_validation'],
                                 stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        stop=node.create_client(Trigger,'/experiment/stop');start=node.create_client(Trigger,'/experiment/start')
        check(stop.wait_for_service(timeout_sec=8),'Recorder stop service did not appear')
        deadline=time.monotonic()+5
        while publishers['pose'].get_subscription_count()==0 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        check(publishers['pose'].get_subscription_count()>0,'Recorder did not subscribe to synthetic pose')
        begin=time.monotonic();begin_ns=time.monotonic_ns();noted=False;aligned=False;warned=False;next_diag=0
        def feed(t):
            nonlocal noted,aligned,warned,next_diag
            stamp=node.get_clock().now().to_msg();sample_us=10000000+int(t*1e6)
            x,y=2+.08*math.sin(2*t),-2+.06*math.cos(t)
            goal_x,goal_y=(2.,-2.) if t<2.3 else (2.1,-2.1)
            p=PoseWithCovarianceStamped();p.header.stamp=stamp;p.header.frame_id='map'
            p.pose.pose.position.x=x;p.pose.pose.position.y=y;p.pose.pose.position.z=.8;publishers['pose'].publish(p)
            u=UwbStatus();u.header=p.header;u.position_valid=True;u.coordinates_confirmed=True;u.raw_position.x=x;u.raw_position.y=-y;u.raw_position.z=.8;u.accepted_rate_hz=50.;publishers['uwb'].publish(u)
            a=AttitudeState();a.header.stamp=stamp;a.header.frame_id='local_ned';a.body_frame_id='base_link_frd';a.system_id=a.component_id=1;a.message_id=31;a.sample_time_us=sample_us if t<3.15 else 100000+int((t-3.15)*1e6);a.valid=True;a.orientation.w=math.sqrt(.5);a.orientation.z=math.sqrt(.5);publishers['att'].publish(a)
            m=BodyMotionState();m.header.stamp=stamp;m.header.frame_id='body_xy';m.system_id=m.component_id=1;m.sample_time_us=sample_us;m.velocity_valid=not (1.1<t<1.4);m.height_valid=m.velocity_valid
            m.forward_velocity_mps=.16*math.cos(2*t) if m.velocity_valid else math.nan;m.right_velocity_mps=.06*math.sin(t) if m.velocity_valid else math.nan;m.height_up_m=1+.05*math.sin(t) if m.height_valid else math.nan;publishers['motion'].publish(m)
            body=BodyVelocitySetpoint();body.header.stamp=stamp;body.header.frame_id='base_link_heading_frd';body.valid=not (1.5<t<1.8 or t>=2.8);body.velocity.x=.42*(goal_x-x) if body.valid else 0.;body.velocity.y=-.42*(goal_y-y) if body.valid else 0.;publishers['body'].publish(body)
            wire=BodyVelocitySetpoint();wire.header.stamp=stamp;wire.header.frame_id='fc_velocity_command';wire.valid=body.valid;wire.velocity.x=body.velocity.x;wire.velocity.y=-body.velocity.y;wire.reason='SYNTHETIC UDP values, not sent to hardware';publishers['sent'].publish(wire)
            n=NavigationStatus();n.header=p.header;n.enabled=t>.4;n.inputs_valid=not (1.5<t<1.8);n.goal_active=.4<t<2.8;n.goal.x=goal_x;n.goal.y=goal_y;n.velocity_map.x=body.velocity.x;n.velocity_map.y=-body.velocity.y;n.distance_m=math.hypot(goal_x-x,goal_y-y);n.actual_speed_mps=math.hypot(m.forward_velocity_mps,m.right_velocity_mps);n.position_age_ms=10.;n.attitude_age_ms=50.;n.feedback_age_ms=50.
            if t<.4:n.state='DISABLED';n.reason='Waiting for start_here'
            elif t<1.5:n.state='ACTIVE';n.reason='XY P/PD active'
            elif t<1.8:n.state='INPUT_HOLD';n.reason='Input recovery: attitude invalid; remaining_ms='+str(int((1.8-t)*1000))
            elif t<2.8:n.state='ACTIVE';n.reason='XY P/PD active'
            else:n.state='FAULT';n.goal_active=False;n.reason='Input recovery window expired; goal cleared'
            publishers['nav'].publish(n)
            velocity=TwistStamped();velocity.header=p.header;velocity.twist.linear=n.velocity_map;publishers['map'].publish(velocity)
            rc=RcOverride();rc.header.stamp=stamp;rc.valid=rc.device_connected=True;rc.channels=[1500]*18;rc.channels[0]=int(1500+100*math.sin(t));publishers['rc'].publish(rc)
            joy=Joy();joy.header.stamp=stamp;joy.axes=[math.sin(t)]+[0.]*7;publishers['joy'].publish(joy)
            raw=MavlinkFrame();raw.header.stamp=stamp;raw.system_id=raw.component_id=1;raw.message_id=250;raw.source_ip='127.0.0.1';raw.fields_json='{"fixture":"synthetic"}';publishers['raw'].publish(raw)
            if t>=.4 and not aligned:
                al=HeadingAlignment();al.header=p.header;al.valid=True;al.quaternion_body_to_ned=True;al.device_time_us=sample_us;al.host_monotonic_ns=time.monotonic_ns();al.host_boot_id='synthetic-host';al.max_clock_skew_ms=1000.;al.reason='Synthetic +X capture';alignment.publish(al);aligned=True
            if t>=.7 and not noted:
                marker=String();marker.data='模拟实验：起飞阶段（测试数据）';publishers['note'].publish(marker);noted=True
            if t>=1.55 and not warned:
                log=Log();log.stamp=stamp;log.level=30;log.name='position_controller_node';log.msg='SYNTHETIC input hold';publishers['rosout'].publish(log);warned=True
            if t>=next_diag:
                d=DiagnosticArray();d.header.stamp=stamp;s=DiagnosticStatus();s.name='mavlink_bridge';s.message='Synthetic traffic'
                for key,value in {'rx_frames':int(t*20),'tx_frames':int(t*30),'velocity_frames':int(t*30),'crc_or_signature_errors':int(t),'tx_errors':0}.items():
                    item=KeyValue();item.key=key;item.value=str(value);s.values.append(item)
                d.status.append(s);publishers['diag'].publish(d);next_diag+=1
        next_frame=begin
        while time.monotonic()-begin<3.6:
            if time.monotonic()>=next_frame:
                feed(time.monotonic()-begin);next_frame+=.02
            rclpy.spin_once(node,timeout_sec=.005)
        def call(client):
            future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=90)
            check(future.done(),'Recorder service timed out');check(future.result().success,'Recorder service failed: '+future.result().message)
            return Path(future.result().message)
        marker_cli=subprocess.run(['bash',str(root/'scripts/flight_log.sh'),'mark','CLI备注验证'],
                                  capture_output=True,text=True,timeout=12)
        check(marker_cli.returncode==0,'Marker CLI failed: '+marker_cli.stderr)
        session=call(stop)
        status_cli=subprocess.run(['bash',str(root/'scripts/flight_log.sh'),'status'],
                                  capture_output=True,text=True,timeout=12)
        check(status_cli.returncode==0,'Status CLI failed: '+status_cli.stderr)
        stopped_status=json.loads(status_cli.stdout)
        check(stopped_status['state']=='finished' and stopped_status['written']>0,'Final recording status lost message counts')
        idle_mark=subprocess.run(['bash',str(root/'scripts/flight_log.sh'),'mark','不应保存'],
                                capture_output=True,text=True,timeout=12)
        check(idle_mark.returncode!=0,'Marker CLI claimed a note was recorded while idle')
        check((session/'report.html').exists(),'HTML report missing')
        check((session/'report'/'flight_report.pdf').read_bytes().startswith(b'%PDF'),'PDF report invalid')
        check(len(list((session/'report').glob('*.png')))==9,'Expected nine plots')
        manifest=json.loads((session/'session.json').read_text());summary=json.loads((session/'summary.json').read_text())
        check(manifest['dropped_messages']==0 and not manifest['recording_error'],'Recording lost messages')
        db=next((session/'bag').glob('*.db3'))
        with sqlite3.connect(str(db)) as connection:
            count=connection.execute('SELECT count(*) FROM messages').fetchone()[0]
        with (session/'timeline.csv').open() as source:timeline=list(csv.DictReader(source))
        check(count==len(timeline)==manifest['written_messages'],'Bag/timeline/manifest counts differ')
        check(summary['active_tracking_samples']>0 and summary['position_rmse_m']>0,'Tracking statistics missing')
        check(summary['input_hold_events']==1 and summary['fault_events']==1,'Event changes were counted as duplicate fault episodes')
        snapshot=json.loads((session/'parameters'/'position_controller_node.json').read_text())
        check(float(snapshot['kp_xy']['value_text'])==.42,'Actual runtime PID snapshot missing')
        with (session/'events.csv').open(encoding='utf-8-sig') as source:events=list(csv.DictReader(source))
        check(any('模拟实验' in e['message'] for e in events),'Chinese operator marker lost')
        check(any('CLI备注验证' in e['message'] for e in events),'Marker CLI was not captured')
        check(any(e['kind']=='goal_accepted' for e in events),'Accepted goal event missing')
        check(any(e['kind']=='device_clock_reset' for e in events),'Clock reset event missing')
        check(any(e['message'].startswith('INPUT_HOLD:') for e in events),'Input hold transition missing')
        with (session/'csv'/'derived_motion.csv').open(encoding='utf-8-sig') as source:motion_rows=list(csv.DictReader(source))
        check(any(row['forward_mps']=='nan' for row in motion_rows),'Invalid flow was fabricated as valid zero')
        with (session/'csv'/'derived_attitude.csv').open(encoding='utf-8-sig') as source:angles=list(csv.DictReader(source))
        check(any(abs(float(row['yaw_map_deg']))<1e-5 for row in angles if row['yaw_map_deg']!='nan'),'Map heading conversion incorrect')
        check(angles[-1]['yaw_map_deg']=='nan','Expired heading was shown as calibrated')
        # Another experiment in the same running process must not overwrite data.
        session2=call(start);feed(4.);rclpy.spin_once(node,timeout_sec=.1);session2_stop=call(stop)
        check(session2==session2_stop and session2!=session,'Separate experiment reused a directory')
        check((session2/'report.html').exists(),'Second report missing')
        second_manifest=json.loads((session2/'session.json').read_text())
        check(second_manifest['topics']['/navigation/heading_alignment']['count']>0,'Next experiment lost the existing alignment reference')
        # SIGINT must finalize and export the final session, including on launch shutdown.
        session3=call(start);feed(4.1)
        until=time.monotonic()+.15
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        process.send_signal(signal.SIGINT);process.wait(timeout=70)
        check(process.returncode==0,'Recorder launch did not shut down cleanly')
        check((session3/'report.html').exists(),'SIGINT did not produce report')
        check(json.loads((session3/'session.json').read_text())['state']=='recorded','SIGINT bag was not finalized')
        return {'status':'passed','hardware_accessed':False,'ros_domain_id':os.environ['ROS_DOMAIN_ID'],
                'synthetic_report':str(session/'report.html'),'synthetic_session':str(session),
                'messages':count,'plots':9,'runtime_parameter_snapshot':True,'raw_nan_preserved':True,
                'map_heading_session_checked':True,'marker_cli':True,'finished_status_counts':True,'alignment_carried_to_next_session':True,'separate_experiments':True,'sigint_auto_export':True}
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=65)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        stream.close();node.destroy_node();rclpy.shutdown()


def main():
    output=Path(__file__).resolve().parents[1]/'log'/'experiment-logging-validation.json'
    try:result=run()
    except Exception as exc:result={'status':'failed','error':str(exc),'hardware_accessed':False}
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(output.read_text())
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':sys.exit(main())
