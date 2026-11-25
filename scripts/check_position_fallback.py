#!/usr/bin/env python3
"""Controlled injected-UWB experiment: real ROS nodes, private DDS, no hardware.
Synthetic stationary truth is used only for assertions, never as flight evidence.
"""
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]

def require(value,message):
    if not value:raise RuntimeError(message)

def rows(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

def run():
    os.environ['ROS_DOMAIN_ID']=str(175+os.getpid()%15);os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from flapping_uav_msgs.msg import UwbStatus,AttitudeState,BodyMotionState,NavigationStatus,BodyVelocitySetpoint,PositionEstimate
    from std_srvs.srv import Trigger
    root=ROOT/'log/experiments_demo'/('fallback_validation_'+str(time.monotonic_ns()));root.mkdir(parents=True)
    rclpy.init();node=rclpy.create_node('fallback_experiment_probe')
    pose=node.create_publisher(PoseWithCovarianceStamped,'/uwb/position',10)
    uwb=node.create_publisher(UwbStatus,'/uwb/status',10)
    att=node.create_publisher(AttitudeState,'/fc/attitude',10)
    motion=node.create_publisher(BodyMotionState,'/fc/body_motion',10)
    last={};history=[];processes=[];streams=[];origin=time.monotonic()
    node.create_subscription(NavigationStatus,'/navigation/status',lambda m:(last.update(nav=m),history.append(m)),50)
    node.create_subscription(BodyVelocitySetpoint,'/fc/velocity_setpoint_body',lambda m:last.update(command=m),50)
    node.create_subscription(PositionEstimate,'/localization/position',lambda m:last.update(estimate=m),50)
    jump=0.;feed_uwb=True;flow_valid=True;reference=0
    def advance(seconds):
        end=time.monotonic()+seconds;next_feed=0
        while time.monotonic()<end:
            if time.monotonic()>=next_feed:
                t=time.monotonic()-origin;sample_us=10000000+int(t*1e6);stamp=node.get_clock().now().to_msg()
                if feed_uwb:
                    p=PoseWithCovarianceStamped();p.header.stamp=stamp;p.header.frame_id='map';p.pose.pose.position.x=2+jump;p.pose.pose.position.y=-2-jump
                    pose.publish(p)
                    u=UwbStatus();u.header=p.header;u.position_valid=True;u.coordinates_confirmed=True;uwb.publish(u)
                a=AttitudeState();a.header.stamp=stamp;a.header.frame_id='local_ned';a.body_frame_id='base_link_frd';a.system_id=a.component_id=1
                a.valid=True;a.angular_velocity_valid=True;a.sample_time_us=sample_us;a.reset_counter=reference
                a.orientation.w=math.sqrt(.5);a.orientation.z=math.sqrt(.5);att.publish(a)
                m=BodyMotionState();m.header.stamp=stamp;m.header.frame_id='body_xy';m.system_id=m.component_id=1;m.sample_time_us=sample_us
                m.velocity_valid=flow_valid;m.forward_velocity_mps=0. if flow_valid else math.nan;m.right_velocity_mps=0. if flow_valid else math.nan
                m.height_valid=True;m.height_up_m=.7;motion.publish(m)
                next_feed=time.monotonic()+.02
            rclpy.spin_once(node,timeout_sec=.004)
    def aligned(client):
        future=client.call_async(Trigger.Request());advance(.3)
        require(future.done() and future.result().success,'start_here failed: '+(future.result().message if future.done() else 'timeout'))
    outputs=[]
    try:
        for enabled in [False,True]:
            last.clear();history.clear();jump=0.;feed_uwb=True;flow_valid=True
            label='flow_assisted' if enabled else 'uwb_only';stream=(ROOT/'log'/('fallback-'+label+'.log')).open('w');streams.append(stream)
            process=subprocess.Popen(['ros2','launch','flapping_uav_bringup','position_hold.launch.py','start_uwb:=false','use_rviz:=false','compact_yaw_only:=false',
                'use_optical_flow_fallback:='+str(enabled).lower(),'log_root:='+str(root),'experiment_label:=synthetic_'+label],
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);processes.append(process)
            client=node.create_client(Trigger,'/navigation/start_here');require(client.wait_for_service(timeout_sec=12),'Navigation unavailable')
            advance(.8);aligned(client);advance(.6)
            require(last['nav'].goal_active and last['command'].valid,'Healthy controller did not activate')
            require(last['nav'].optical_flow_fallback_enabled==enabled,'Wrong experimental mode')
            start_syncs=last['estimate'].uwb_syncs;advance(.6)
            require(0<last['estimate'].uwb_syncs-start_syncs<=6,'Flow odometry was not periodically synchronized')
            jump=2.;advance(.7)
            if enabled:
                require(last['nav'].position_source=='FLOW_ODOMETRY','UWB jump did not trigger flow control')
                require(last['command'].valid and abs(last['nav'].position_used.x-2)<.03,'Bad UWB entered the assisted controller')
                require(abs(last['command'].velocity.x)<.05 and abs(last['command'].velocity.y)<.05,'Stationary assisted controller chased injected outlier')
                require(last['estimate'].reliable_uwb_age_ms>500,'Rejected UWB renewed the reliable anchor clock')
            else:
                require(last['nav'].position_source=='UWB' and abs(last['nav'].position_used.x-2)<.03,'Filtered UWB baseline consumed a rejected raw jump')
                require(not last['command'].valid and last['nav'].state=='INPUT_HOLD','Without flow fallback, rejected UWB must stop control')
            feed_uwb=False;advance(2.2 if enabled else .2)
            if enabled:
                require(last['command'].valid and last['nav'].goal_active,'UWB loss cleared the assisted goal within DR horizon')
                require(last['nav'].position_source=='FLOW_ODOMETRY','Dropout did not retain optical odometry')
            jump=0.;feed_uwb=True;advance(.2)
            if enabled:require(last['nav'].position_source=='FLOW_ODOMETRY','A few good points caused premature switch-back')
            advance(.45)
            if enabled:require(last['nav'].position_source=='UWB_BLEND','Recovery did not enter the configured blend')
            advance(.65)
            require(last['command'].valid and last['nav'].goal_active,'Recovered data did not resume the original target')
            if enabled:
                require(last['nav'].position_source=='UWB','Stable UWB was not restored')
                jump=2.;advance(.15);flow_valid=False;advance(.15)
                require(not last['command'].valid and last['nav'].state=='INPUT_HOLD','Invalid flow was treated as usable zero odometry')
                flow_valid=True;advance(.15);require(not last['command'].valid,'A missing flow interval silently regained a position')
                jump=0.;advance(1.2);require(last['command'].valid,'UWB did not re-anchor after invalid optical flow')
                jump=2.;advance(5.3)
                require(not last['command'].valid,'Dead reckoning exceeded 5 seconds')
                require(last['nav'].state=='INPUT_HOLD','Expired position did not enter existing zero-output recovery')
                advance(2.2);require(last['nav'].state=='FAULT' and not last['nav'].goal_active,'Persistent fault did not clear target')
                jump=0.;advance(1.2);require(not last['nav'].goal_active,'Old target automatically resurrected after latched fault')
                aligned(client);advance(.6);reference+=1;advance(.15)
                require(not last['nav'].goal_active and not last['command'].valid,'FC reference reset reused the old goal')
            process.send_signal(signal.SIGINT);process.wait(timeout=90);require(process.returncode==0,'Navigation did not finalize cleanly')
            matching=list(root.glob('*synthetic_'+label+'*/session.json'));require(len(matching)==1,'Experiment was not recorded automatically')
            session=matching[0].parent;manifest=json.loads(matching[0].read_text())
            require(manifest['flow_assistance_enabled']==enabled,'Recorded mode does not match actual controller')
            require(not (session/'bag').exists(),'Unexpected bag recording')
            used=rows(session/'control_position.csv');sources=rows(session/'position_sources.csv')
            require(used and sources,'Paper CSV is missing controller/source data')
            same=session/'localization_comparison'
            require(len(list(same.glob('*.png')))==2 and len(list(same.glob('*.pdf')))==2,'Same-flight localization figures missing')
            require((same/'uwb_raw_points.csv').read_bytes()==(session/'trajectory.csv').read_bytes(),'Raw comparison points changed')
            exported=rows(same/'flow_compensated_points.csv')
            require(len(exported)==len(sources) and all(a['selected_x_m']==b['x_m'] and a['selected_y_m']==b['y_m'] and a['valid']==b['valid'] for a,b in zip(sources,exported)),
                    'Compensated comparison points changed')
            if enabled:
                require({'UWB','FLOW_ODOMETRY','UWB_BLEND','INVALID'}<={r['source'] for r in used},'Source transitions not retained')
                require(json.loads((same/'comparison.json').read_text())['flow_fallback_observed'],'Recorded flow takeover not shown in same-flight comparison')
            outputs.append(session)
            node.destroy_client(client)
            until=time.monotonic()+.25
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        result=subprocess.run(['bash','scripts/flight_log.sh','compare-positions',str(outputs[0]),str(outputs[1])],capture_output=True,text=True,timeout=90)
        require(result.returncode==0,'Position comparison export failed: '+result.stderr)
        compare=outputs[1]/'position_comparison'
        require(len(list(compare.glob('*.png')))==2 and len(list(compare.glob('*.pdf')))==2,'Expected two comparison figures')
        require((compare/'uwb_only_position.csv').read_bytes()==(outputs[0]/'control_position.csv').read_bytes(),'Baseline comparison altered samples')
        require((compare/'flow_assisted_position.csv').read_bytes()==(outputs[1]/'control_position.csv').read_bytes(),'Assisted comparison altered samples')
        return {'status':'passed','hardware_accessed':False,'data_kind':'SYNTHETIC UWB FAULT INJECTION; NOT FLIGHT EVIDENCE',
            'sessions':[str(s) for s in outputs],'comparison':str(compare),'ros_domain_id':os.environ['ROS_DOMAIN_ID'],
            'checked':['actual controller A/B modes','periodic odometry sync','jump isolation','dropout beyond 2s retains target',
                'recovery time and sample hysteresis','blended UWB return','invalid-flow stop','no missing-interval integration',
                '5s dead-reckoning limit','persistent fault latches','FC reset clears target','automatic CSV recording',
                'same-flight raw/compensated figures and unchanged points','two unmodified position figures']}
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=90)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    try:result=run()
    except Exception as exc:result={'status':'failed','error':str(exc),'hardware_accessed':False}
    output=ROOT/'log/position-fallback-validation.json';output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(output.read_text())
    raise SystemExit(0 if result['status']=='passed' else 1)
