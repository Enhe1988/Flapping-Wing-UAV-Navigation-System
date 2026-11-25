#!/usr/bin/env python3
"""Private ROS navigation with a synthetic XY velocity-loop plant; no hardware."""
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
ROOT=Path(__file__).resolve().parents[1]

def require(value,message):
    if not value:raise RuntimeError(message)

def rows(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

def run():
    os.environ['ROS_DOMAIN_ID']=str(150+os.getpid()%20);os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
    from flapping_uav_msgs.msg import AttitudeState,BodyMotionState,UwbStatus,NavigationStatus,BodyVelocitySetpoint
    from visualization_msgs.msg import MarkerArray
    from std_srvs.srv import Trigger
    root=ROOT/'log/experiments_demo'/('trajectory_validation_'+str(time.monotonic_ns()));root.mkdir(parents=True)
    config=yaml.safe_load((ROOT/'src/flapping_uav_bringup/config/trajectory.yaml').read_text())
    config['trajectory'].update(figure8_fit_scale=.12,line_length_m=.6,line_period_s=3.,figure8_period_s=4.,ramp_s=1.,
                              max_reference_speed_mps=.4,max_reference_acceleration_mps2=.6,min_approach_time_s=.5)
    config_path=root/'synthetic_trajectory.yaml';config_path.write_text(yaml.safe_dump(config))
    spec=importlib.util.spec_from_file_location('trajectory_geometry',ROOT/'src/flapping_uav_bringup/launch/trajectory_geometry.py')
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    scene=yaml.safe_load((ROOT/'src/flapping_uav_bringup/config/scene.yaml').read_text())['scene_visualizer_node']['ros__parameters']
    uwb_config=yaml.safe_load((ROOT/'src/flapping_uav_bringup/config/uwb.yaml').read_text())['uwb_position_node']['ros__parameters']
    geometry=helper.geometry(scene,uwb_config,config['trajectory']['boundary_margin_m'],.12,.6,
                             config['trajectory'].get('figure8_axis','x'),config['trajectory'].get('figure8_lobe_aspect_ratio'))
    # Start off the path, relative to today's map instead of old anchor coordinates.
    center=geometry['center'];initial=[center[0],center[1]+.4]
    rclpy.init();node=rclpy.create_node('trajectory_plant_probe')
    pose=node.create_publisher(PoseWithCovarianceStamped,'/uwb/position',10)
    uwb=node.create_publisher(UwbStatus,'/uwb/status',10)
    att=node.create_publisher(AttitudeState,'/fc/attitude',10)
    flow=node.create_publisher(BodyMotionState,'/fc/body_motion',10)
    goals=node.create_publisher(PoseStamped,'/navigation/goal',10)
    last={};history=[];previews=[];processes=[];streams=[];origin=time.monotonic();last_step=time.monotonic()
    position=list(initial);velocity=[0.,0.];bad_flow=False;bad_uwb=False
    freeze_plant=False;reported_velocity=None
    node.create_subscription(NavigationStatus,'/navigation/status',lambda m:(last.update(nav=m),history.append(m)),100)
    node.create_subscription(BodyVelocitySetpoint,'/fc/velocity_setpoint_body',lambda m:last.update(command=m),100)
    node.create_subscription(MarkerArray,'/navigation/trajectory_markers',lambda m:previews.append(m),10)
    def advance(seconds):
        nonlocal last_step
        end=time.monotonic()+seconds;next_feed=0
        while time.monotonic()<end:
            if time.monotonic()>=next_feed:
                now=time.monotonic();dt=min(.05,now-last_step);last_step=now
                cmd=last.get('command');target=[cmd.velocity.x,-cmd.velocity.y] if cmd and cmd.valid else [0.,0.]
                gain=1-math.exp(-dt/.12)
                if freeze_plant:velocity[:]=[0.,0.]
                else:
                    for i in [0,1]:velocity[i]+=gain*(target[i]-velocity[i]);position[i]+=velocity[i]*dt
                t=now-origin;stamp=node.get_clock().now().to_msg();sample=10000000+int(t*1e6)
                p=PoseWithCovarianceStamped();p.header.stamp=stamp;p.header.frame_id='map'
                p.pose.pose.position.x=position[0]+(2. if bad_uwb else 0.);p.pose.pose.position.y=position[1];p.pose.pose.position.z=42.;pose.publish(p)
                u=UwbStatus();u.header=p.header;u.position_valid=True;u.coordinates_confirmed=True;uwb.publish(u)
                a=AttitudeState();a.header.stamp=stamp;a.header.frame_id='local_ned';a.body_frame_id='base_link_frd';a.system_id=a.component_id=1
                a.valid=True;a.sample_time_us=sample;a.orientation.w=math.sqrt(.5);a.orientation.z=math.sqrt(.5);att.publish(a)
                m=BodyMotionState();m.header.stamp=stamp;m.header.frame_id='body_xy';m.system_id=m.component_id=1;m.sample_time_us=sample
                measured=reported_velocity if reported_velocity is not None else velocity
                m.velocity_valid=not bad_flow;m.forward_velocity_mps=measured[0] if not bad_flow else math.nan;m.right_velocity_mps=-measured[1] if not bad_flow else math.nan
                m.height_up_m=.8;m.height_valid=True;flow.publish(m);next_feed=now+.02
            rclpy.spin_once(node,timeout_sec=.003)
    def call(client):
        future=client.call_async(Trigger.Request());advance(.2)
        require(future.done(),'Service timed out');return future.result()
    def wait_phase(phase,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if last['nav'].trajectory_phase==phase:return
            advance(.05)
        raise RuntimeError('Did not reach '+phase+': '+last['nav'].trajectory_phase)
    sessions=[]
    try:
        for mode in ['line','figure8']:
            position[:]=initial;velocity[:]=[0.,0.];last.clear();history.clear();previews.clear();bad_flow=bad_uwb=freeze_plant=False;reported_velocity=None
            stream=(ROOT/'log'/('trajectory-'+mode+'.log')).open('w');streams.append(stream)
            process=subprocess.Popen(['ros2','launch','flapping_uav_bringup','position_hold.launch.py','start_uwb:=false','use_rviz:=false','compact_yaw_only:=false',
                'use_optical_flow_fallback:=true',
                'flight_mode:='+mode,'trajectory_config:='+str(config_path),'log_root:='+str(root),'experiment_label:=synthetic_'+mode],
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);processes.append(process)
            align=node.create_client(Trigger,'/navigation/start_here');start=node.create_client(Trigger,'/navigation/start_trajectory');stop=node.create_client(Trigger,'/navigation/stop_trajectory')
            require(align.wait_for_service(timeout_sec=12) and start.wait_for_service(timeout_sec=2),'Navigation services missing')
            advance(.7);require(not call(start).success,'Trajectory started before alignment')
            require(call(align).success,'Could not align synthetic aircraft');advance(.5)
            require(last['nav'].flight_mode==mode and not last['nav'].trajectory_running,'Alignment auto-started a trajectory')
            held=(last['nav'].goal.x,last['nav'].goal.y);advance(.4)
            require((last['nav'].goal.x,last['nav'].goal.y)==held,'Reference moved before start_trajectory')
            bad_flow=True;advance(.1)
            require(not call(start).success,'Invalid optical-flow velocity started a trajectory')
            bad_flow=False;freeze_plant=True;reported_velocity=[.6,0.];advance(.1)
            require(last['nav'].actual_speed_mps>.5,'Moving-start fixture did not exceed the former threshold')
            started=call(start)
            require(started.success,'Valid velocity above the former threshold could not start trajectory')
            approach_seconds=float(started.message.split('approach=')[1].split('s;')[0])
            reported_velocity=None
            require(not call(start).success,'Double start reset a running trajectory')
            advance(approach_seconds+.2)
            require(last['nav'].trajectory_phase=='APPROACH','Started tracing before actual arrival at the start point')
            require(math.hypot(last['nav'].goal.x-position[0],last['nav'].goal.y-position[1])>.1,'Off-start fixture was already at the start')
            waiting=last['nav'].trajectory_time_s;advance(.2)
            require(abs(last['nav'].trajectory_time_s-waiting)<1e-9,'Approach clock did not wait for actual arrival')
            require(not any(m.trajectory_phase=='TRACKING' for m in history),'Tracking occurred during blocked approach')
            freeze_plant=False;wait_phase('TRACKING',10)
            tracking=[m for m in history if m.trajectory_phase=='TRACKING']
            require(tracking and math.hypot(tracking[0].position_used.x-tracking[0].goal.x,
                    tracking[0].position_used.y-tracking[0].goal.y)<config['trajectory'].get('start_position_tolerance_m',.05)+.01,
                    'Tracking did not begin within the configured start position tolerance')
            initial_time=last['nav'].trajectory_time_s
            bad_uwb=True;advance(.4)
            require(last['nav'].position_source=='FLOW_ODOMETRY' and last['nav'].trajectory_time_s>initial_time+.2,'Valid fallback did not continue the trajectory')
            bad_uwb=False;advance(1.2)
            if mode=='figure8':
                bad_flow=True;advance(.15)
                require(last['nav'].state=='INPUT_HOLD','Invalid measured velocity did not pause tracking while UWB was healthy')
                paused=last['nav'].trajectory_time_s;advance(.15)
                require(abs(last['nav'].trajectory_time_s-paused)<1e-9,'Trajectory clock moved with invalid measured velocity')
                require(abs(last['nav'].trajectory_velocity_map.x)+abs(last['nav'].trajectory_velocity_map.y)<1e-12,'Paused reference velocity was not zero')
                bad_flow=False;advance(.3)
                bad_uwb=True;bad_flow=True;advance(.15)
                paused=last['nav'].trajectory_time_s;require(last['nav'].state=='INPUT_HOLD','Bad inputs did not pause trajectory')
                advance(.2);require(abs(last['nav'].trajectory_time_s-paused)<1e-9,'Trajectory clock advanced with invalid position')
                bad_uwb=False;bad_flow=False;advance(1.25)
                require(last['nav'].trajectory_running and last['command'].valid,'Trajectory did not resume after input recovery')
            wait_phase('FINISHED',30);advance(.6)
            require(not last['nav'].trajectory_running,'Finished trajectory remained active')
            require(abs(last['nav'].trajectory_velocity_map.x)+abs(last['nav'].trajectory_velocity_map.y)<1e-12,'Terminal reference velocity was nonzero')
            moving=[m for m in history if m.trajectory_running and m.inputs_valid]
            require(moving and all(abs(m.goal.z)<1e-12 and abs(m.trajectory_velocity_map.z)<1e-12 for m in moving),'Horizontal trajectory issued a height reference')
            require(all(abs(last['command'].velocity.z)<1e-12 for _ in [0]),'Vertical command produced')
            require(any(math.hypot(m.trajectory_velocity_map.x,m.trajectory_velocity_map.y)>.02 for m in moving),'Analytic velocity feedforward missing')
            tracking=[m for m in moving if m.trajectory_phase=='TRACKING']
            require(max(math.hypot(m.goal.x-m.position_used.x,m.goal.y-m.position_used.y) for m in tracking)<.5,'Synthetic tracking error unexpectedly large')
            require(previews and all(abs(p.z)<1e-12 for array in previews for marker in array.markers for p in marker.points),'RViz plan was not horizontal')
            if mode=='line':require(max(m.goal.y for m in tracking)-min(m.goal.y for m in tracking)<.01,'Line left its fixed map Y')
            else:require(max(m.goal.y for m in moving)-min(m.goal.y for m in moving)>.35,'Figure eight did not span map Y')
            advance(.5);require(call(start).success,'Completed trajectory could not restart');advance(.25)
            require(call(stop).success,'Manual stop failed');require(not last['nav'].trajectory_running and last['nav'].goal_active,'Stop did not hold current XY')
            advance(.8);require(call(start).success,'Stopped trajectory could not restart');advance(.2)
            goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=position[0];goal.pose.position.y=position[1];goal.pose.orientation.w=1.
            goals.publish(goal);advance(.15);require(not last['nav'].trajectory_running,'Manual point target did not cancel trajectory')
            process.send_signal(signal.SIGINT);process.wait(timeout=90);require(process.returncode==0,'Navigation shutdown failed')
            matches=list(root.glob('*synthetic_'+mode+'*/session.json'));require(len(matches)==1,'Automatic trajectory experiment missing')
            session=matches[0].parent;manifest=json.loads(matches[0].read_text());require(manifest['flight_mode']==mode,'Mission metadata lost')
            reference=rows(session/'reference.csv');events=rows(session/'events.csv')
            require({'TRACKING','FINISHED','CANCELLED'}<={r['trajectory_phase'] for r in reference},'Recorded mission phases missing')
            require(reference[0]['trajectory_phase']=='TRACKING','Approach leaked before first TRACKING')
            require(manifest['recording_start_trigger']=='trajectory_tracking','Wrong recording start trigger')
            require(len([e for e in events if e['event']=='goal'])<20,'Every trajectory sample was mislabeled as a waypoint event')
            require(len(list((session/'report').glob('*.png')))==5,'Expected legacy figures plus raw/filtered UWB pair')
            require(not (session/'bag').exists(),'Unexpected bag recording');sessions.append(str(session))
            for client in [align,start,stop]:node.destroy_client(client)
            advance(.2)
        return {'status':'passed','hardware_accessed':False,'data_kind':'SYNTHETIC XY PLANT; NOT FLIGHT EVIDENCE','sessions':sessions,
                'checked':['manual start after heading alignment','invalid flow rejected at start','valid moving start above former speed threshold',
                           'off-start approach waits for actual XY arrival','smooth approach','line and horizontal figure eight','analytic feedforward',
                           'finite completion and position hold','fallback continues trajectory','invalid inputs freeze trajectory clock',
                           'manual stop and restart','manual target cancels trajectory','zero Z command','sparse events and raw reference CSV','automatic paper export']}
    finally:
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=90)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':
    try:result=run()
    except Exception as exc:result={'status':'failed','error':str(exc),'hardware_accessed':False}
    path=ROOT/'log/trajectory-flight-validation.json';path.parent.mkdir(exist_ok=True);path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(path.read_text())
    raise SystemExit(0 if result['status']=='passed' else 1)
