#!/usr/bin/env python3
"""Synthetic private-DDS independent fusion observation check. No FC/RC commands."""
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def require(value, message):
    if not value:raise RuntimeError(message)


def run():
    os.environ['ROS_DOMAIN_ID']=str(211+os.getpid()%15)
    os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from flapping_uav_msgs.msg import AttitudeState,BodyMotionState,FusionStatus,UwbStatus
    root=Path(__file__).resolve().parents[1]
    rclpy.init();node=rclpy.create_node('fusion_observer_synthetic_probe')
    pose=node.create_publisher(PoseWithCovarianceStamped,'/uwb/position',10)
    uwb=node.create_publisher(UwbStatus,'/uwb/status',10)
    attitude=node.create_publisher(AttitudeState,'/fc/attitude',10)
    flow=node.create_publisher(BodyMotionState,'/fc/body_motion',10)
    last={};odoms=[]
    subs=[node.create_subscription(FusionStatus,'/fusion/status',lambda m:last.update(status=m),20),
          node.create_subscription(Odometry,'/fusion/odometry',lambda m:odoms.append(m),100)]
    processes=[];streams=[]
    origin=time.monotonic();feed_uwb=True;valid_flow=True;jump=0.;reference=0
    def advance(seconds):
        end=time.monotonic()+seconds;next_feed=0
        while time.monotonic()<end:
            if time.monotonic()>=next_feed:
                t=time.monotonic()-origin;stamp=node.get_clock().now().to_msg();sample=10000000+int(t*1e6)
                if feed_uwb:
                    p=PoseWithCovarianceStamped();p.header.stamp=stamp;p.header.frame_id='map'
                    p.pose.pose.position.x=1+.2*t+jump;p.pose.pose.position.y=-2-.05*t;p.pose.pose.position.z=99.
                    pose.publish(p)
                    u=UwbStatus();u.header=p.header;u.position_valid=True;u.coordinates_confirmed=True;uwb.publish(u)
                a=AttitudeState();a.header.stamp=stamp;a.header.frame_id='local_ned';a.body_frame_id='base_link_frd'
                a.system_id=a.component_id=1;a.sample_time_us=sample;a.reset_counter=reference;a.valid=True;a.orientation.w=1.;attitude.publish(a)
                m=BodyMotionState();m.header.stamp=stamp;m.header.frame_id='body_xy';m.system_id=m.component_id=1
                m.sample_time_us=sample;m.velocity_valid=valid_flow;m.height_valid=False
                m.forward_velocity_mps=.2 if valid_flow else math.nan;m.right_velocity_mps=.05 if valid_flow else math.nan
                m.height_up_m=math.nan;flow.publish(m)
                next_feed=time.monotonic()+.02
            rclpy.spin_once(node,timeout_sec=.004)
    try:
        for label,args in [('fusion',['fusion_odometry.launch.py'])]:
            stream=(root/'log'/('fusion-observer-'+label+'.log')).open('w');streams.append(stream)
            processes.append(subprocess.Popen(['ros2','launch','flapping_uav_bringup']+args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
        align=node.create_client(Trigger,'/fusion/align_positive_x')
        require(align.wait_for_service(timeout_sec=8),'Observer service missing')
        advance(.5);require(last['status'].state=='UWB_ONLY','UWB-only initialization failed')
        future=align.call_async(Trigger.Request());advance(.15);require(future.done() and future.result().success,'Independent +X alignment failed')
        advance(.25);require(last['status'].state=='FUSED' and last['status'].valid,'Healthy fusion failed')
        require(not any(name=='/navigation/start_here' for name,_ in node.get_service_names_and_types()),'Observer unexpectedly started navigation')
        jump=4.;advance(.3)
        require(last['status'].state=='DEAD_RECKONING' and last['status'].rejected_uwb>0,'UWB jump was not rejected')
        feed_uwb=False;advance(.6)
        require(last['status'].state=='DEAD_RECKONING' and last['status'].valid,'Short flow-only propagation failed')
        t=time.monotonic()-origin
        require(abs(last['status'].position.x-(1+.2*t))<.1,'Forward flow integration direction/scale incorrect')
        require(abs(last['status'].position.y-(-2-.05*t))<.1,'Rightward flow map conversion incorrect')
        advance(5.2);require(last['status'].state=='INVALID' and not last['status'].valid,'Fallback exceeded its horizon')
        count=len(odoms);advance(.12);require(len(odoms)==count,'Invalid estimate still refreshed Odometry')
        jump=0.;feed_uwb=True;advance(.8);require(last['status'].state=='FUSED' and last['status'].valid,'Consistent UWB recovery failed')
        valid_flow=False;advance(.25);require(last['status'].state=='UWB_ONLY' and last['status'].valid,'Invalid ground flow incorrectly disabled UWB')
        feed_uwb=False;advance(.6);require(not last['status'].valid,'Invalid flow fabricated a zero-velocity bridge')
        valid_flow=True;advance(.2);require(not last['status'].valid,'Flow gap silently regained a valid trajectory without UWB')
        feed_uwb=True;advance(.8);require(last['status'].state=='FUSED','UWB did not re-anchor after flow gap')
        reference+=1;advance(.12);require(not last['status'].alignment_valid and last['status'].state=='UWB_ONLY','FC reset reused old heading')
        require(node.count_publishers('/fc/velocity_setpoint_body')==0,'Observer emitted FC control')
        require(node.count_publishers('/fc/velocity_setpoint_sent')==0,'Observer emitted wire commands')
        require(node.count_publishers('/navigation/goal')==0,'Observer emitted navigation goals')
        return {'status':'passed','hardware_accessed':False,'navigation_connected':False,'control_publishers':0,
                'checked':['UWB-only ground mode','independent heading alignment','healthy fusion','jump rejection',
                           'short dead reckoning','5s horizon','suppressed invalid odometry','confirmed recovery',
                           'NaN flow','no integration across missing interval','FC reference reset']}
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=70)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()


def main():
    output=Path(__file__).resolve().parents[1]/'log'/'fusion-observer-validation.json'
    try:result=run()
    except Exception as exc:result={'status':'failed','error':str(exc),'hardware_accessed':False}
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(output.read_text())
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
