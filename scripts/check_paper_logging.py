#!/usr/bin/env python3
"""Private-DDS CSV logging check with real navigation and loopback MAVLink31/250."""
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]

def require(value,message):
    if not value:raise RuntimeError(message)

def read_rows(path):
    with path.open(encoding='utf-8-sig',newline='') as stream:return list(csv.DictReader(stream))

def frame(message_id,payload,extra,sequence):
    header=bytes([len(payload),sequence%256,1,1,message_id]);crc=65535
    for value in header+payload+bytes([extra]):
        tmp=value^(crc&255);tmp^=(tmp<<4)&255
        crc=((crc>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&65535
    return b'\xfe'+header+payload+struct.pack('<H',crc)

def descendants(pid):
    found=[]
    try:children=Path('/proc/{}/task/{}/children'.format(pid,pid)).read_text().split()
    except OSError:return found
    for child in children:
        child=int(child);found.append(child);found.extend(descendants(child))
    return found

def run():
    os.environ['ROS_DOMAIN_ID']=str(201+os.getpid()%25);os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
    from flapping_uav_msgs.msg import UwbStatus,HeadingAlignment,AttitudeState,BodyVelocitySetpoint
    from std_srvs.srv import Trigger
    root=ROOT/'log/experiments_demo'/('paper_validation_'+str(time.monotonic_ns()))
    root.mkdir(parents=True)
    rclpy.init();node=rclpy.create_node('paper_log_probe')
    pose=node.create_publisher(PoseWithCovarianceStamped,'/uwb/position',10)
    uwb=node.create_publisher(UwbStatus,'/uwb/status',10)
    goals=node.create_publisher(PoseStamped,'/navigation/goal',10)
    old_alignment=node.create_publisher(HeadingAlignment,'/navigation/heading_alignment',10)
    wire_monitor=node.create_publisher(BodyVelocitySetpoint,'/fc/velocity_setpoint_sent',10)
    observed=[];node.create_subscription(AttitudeState,'/fc/attitude',lambda m:observed.append(m),100)
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    reserve=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);reserve.bind(('127.0.0.1',0));port=reserve.getsockname()[1];reserve.close()
    streams=[];processes=[];source_positions={};origin=time.monotonic();seq=0;bad_flow=False;bad_rate=False
    def feed(seconds):
        nonlocal seq
        end=time.monotonic()+seconds;next_sample=0
        while time.monotonic()<end:
            if time.monotonic()>=next_sample:
                t=time.monotonic()-origin;stamp=node.get_clock().now().to_msg();sample_us=10000000+int(t*1e6)
                p=PoseWithCovarianceStamped();p.header.stamp=stamp;p.header.frame_id='map'
                p.pose.pose.position.x=2+.03*t+.005*math.sin(25*t);p.pose.pose.position.y=-2+.01*math.cos(23*t);p.pose.pose.position.z=.8
                source_positions[stamp.sec*1000000000+stamp.nanosec]=(p.pose.pose.position.x,p.pose.pose.position.y)
                pose.publish(p)
                u=UwbStatus();u.header=p.header;u.position_valid=True;u.coordinates_confirmed=True;uwb.publish(u)
                q=math.sqrt(.5)
                payload=struct.pack('<I7f',sample_us//1000,q,0.,0.,q,math.nan if bad_rate else .1,-.2,.3)
                sender.sendto(frame(31,payload,246,seq),('127.0.0.1',port));seq+=1
                payload=struct.pack('<Q3f10s',sample_us,math.nan if bad_flow else .2,-.1,.7,b'FLOW_EST')
                sender.sendto(frame(250,payload,49,seq),('127.0.0.1',port));seq+=1
                sent=BodyVelocitySetpoint();sent.header.stamp=stamp;sent.header.frame_id='fc_velocity_command'
                sent.valid=True;sent.velocity.x=.123456789;sent.velocity.y=-.234567891
                sent.command_age_ms=17.;sent.attitude_age_ms=23.;sent.reason='Synthetic send-monitor sample; no hardware'
                wire_monitor.publish(sent)
                next_sample=time.monotonic()+.02
            rclpy.spin_once(node,timeout_sec=.004)
    def launch(label,args):
        stream=(ROOT/'log'/('paper-'+label+'.log')).open('w');streams.append(stream)
        process=subprocess.Popen(['ros2']+args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    def start_navigation(label):
        process=launch(label,['launch','flapping_uav_bringup','position_hold.launch.py','start_uwb:=false','use_rviz:=false','compact_yaw_only:=false',
                              'log_root:='+str(root),'experiment_label:='+label])
        client=node.create_client(Trigger,'/navigation/start_here')
        require(client.wait_for_service(timeout_sec=12),'Navigation service unavailable')
        feed(.15)
        return process,client
    def align(client):
        future=client.call_async(Trigger.Request());feed(.25)
        require(future.done() and future.result().success,'Heading alignment did not succeed')
    def check_session(session):
        m=json.loads((session/'session.json').read_text());summary=json.loads((session/'summary.json').read_text())
        require(m['format']=='paper_csv_v1' and m['state']=='recorded','CSV session did not finalize')
        require(not (session/'bag').exists() and not list(session.rglob('*.db3')),'A rosbag was created')
        require(not m['dropped_messages'] and not m['recording_error'],'CSV writer lost data')
        require(not summary['missing_data'] and not summary['row_count_mismatches'],'Required raw table missing/incomplete')
        for name,meta in m['files'].items():require(len(read_rows(session/name))==meta['rows'],'Row count mismatch: '+name)
        require(len(list((session/'report').glob('*.png')))==5,'Expected legacy figures plus raw/filtered UWB pair')
        require(len(list((session/'report').glob('*.pdf')))==5,'Expected legacy PDFs plus raw/filtered UWB pair')
        comparison=session/'localization_comparison'
        require(len(list(comparison.glob('*.png')))==2 and len(list(comparison.glob('*.pdf')))==2,'Same-flight comparison figures missing')
        require((comparison/'uwb_raw_points.csv').read_bytes()==(session/'trajectory.csv').read_bytes(),'Same-flight raw UWB copy changed samples')
        source=read_rows(session/'position_sources.csv');compensated=read_rows(comparison/'flow_compensated_points.csv')
        require(len(source)==len(compensated) and all(a['selected_x_m']==b['x_m'] and a['selected_y_m']==b['y_m'] and a['valid']==b['valid'] for a,b in zip(source,compensated)),
                'Same-flight compensated copy changed points or validity')
        for p in (session/'report').glob('*.pdf'):require(p.read_bytes().startswith(b'%PDF'),'Invalid PDF')
        for row in read_rows(session/'trajectory.csv'):
            expected=source_positions[int(row['source_ros_ns'])]
            require(abs(float(row['x_m'])-expected[0])<1e-12 and abs(float(row['y_m'])-expected[1])<1e-12,'Raw position was modified/filtered')
        filtered=read_rows(session/'uwb_filtered.csv');require(filtered,'Online filtered UWB CSV is empty')
        require(all(int(row['source_ros_ns']) in source_positions for row in filtered),'Filtered fix lost its original observation stamp')
        require(all(abs(float(row['z_uwb_raw_m'])-.8)<1e-12 for row in filtered),'XY filter changed raw Z')
        require(any(abs(float(row['x_m'])-source_positions[int(row['source_ros_ns'])][0])>1e-4 for row in filtered),
                'Configured online low-pass did not change the synthetic noisy measurements')
        a=read_rows(session/'attitude.csv');require(any(abs(float(r['p_rad_s'])-.1)<1e-6 for r in a),'ID31 roll speed lost')
        require(any(abs(float(r['q_rad_s'])+.2)<1e-6 and abs(float(r['r_rad_s'])-.3)<1e-6 for r in a),'ID31 pitch/yaw speeds lost')
        require(any(abs(float(r['yaw_map_deg']))<1e-4 for r in a),'Aligned map yaw incorrect')
        sent=read_rows(session/'velocity_sent.csv')
        require(sent and all(abs(float(r['forward_wire_mps'])-.123456789)<1e-12 and
                abs(float(r['lateral_wire_mps'])+.234567891)<1e-12 and r['valid']=='1' and
                r['frame_id']=='fc_velocity_command' and float(r['command_age_ms'])==17. and
                float(r['attitude_age_ms'])==23. for r in sent),'Packed send monitor values/ages were not retained')
        return m
    try:
        launch('bridge',['run','flapping_uav_gcs','mavlink_bridge_node','--ros-args','-p','bind_ip:=127.0.0.1','-p','rx_port:='+str(port),
                         '-p','tx_enabled:=false','-p','print_rx:=false'])
        nav,client=start_navigation('launch_ctrl_c')
        require(not list(root.glob('*/session.json')),'Recording started before alignment')
        stale=HeadingAlignment();stale.header.stamp=node.get_clock().now().to_msg();stale.header.stamp.sec-=30;stale.header.frame_id='map';stale.valid=True
        stale.device_time_us=1;stale.host_monotonic_ns=time.monotonic_ns();stale.host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        old_alignment.publish(stale);feed(.15);require(not list(root.glob('*/session.json')),'Stale retained alignment started recording')
        align(client);feed(.4)
        require(len(list(root.glob('*/session.json')))==1,'Alignment did not automatically start one session')
        target=PoseStamped();target.header.frame_id='map';target.pose.position.x=2.5;target.pose.position.y=-1.8;target.pose.orientation.w=1.
        goals.publish(target);feed(.5);bad_flow=True;bad_rate=True;feed(.25);bad_flow=False;bad_rate=False;feed(.3)
        align(client);feed(.3)
        require(len(list(root.glob('*/session.json')))==1,'Realignment overwrote/split the active recording')
        nav.send_signal(signal.SIGINT);nav.wait(timeout=90);require(nav.returncode==0,'Navigation launch did not shut down cleanly')
        session=next(root.glob('*/session.json')).parent;m=check_session(session)
        require(any(r['velocity_valid']=='0' and math.isnan(float(r['forward_mps'])) for r in read_rows(session/'velocity_actual.csv')),'Invalid flow fabricated as zero')
        require(any(r['angular_velocity_valid']=='0' and math.isnan(float(r['p_rad_s'])) for r in read_rows(session/'attitude.csv')),'Invalid body rate lost')
        require(any(abs(float(r['goal_x_m'])-2.5)<1e-12 for r in read_rows(session/'reference.csv')),'Accepted desired position missing')
        nav2,client2=start_navigation('node_exit');align(client2);feed(.5)
        controller=[]
        for pid in descendants(nav2.pid):
            try:command=Path('/proc/{}/cmdline'.format(pid)).read_bytes().split(b'\0')[0]
            except OSError:continue
            if command.endswith(b'/position_controller_node'):controller.append(pid)
        require(len(controller)==1,'Could not locate the private navigation child')
        os.kill(controller[0],signal.SIGINT);nav2.wait(timeout=90)
        sessions=sorted(p.parent for p in root.glob('*/session.json'));require(len(sessions)==2,'Second session missing')
        for s in sessions:check_session(s)
        require(any(a.angular_velocity_valid for a in observed),'MAVLink bridge did not expose angular rates')
        return {'status':'passed','hardware_accessed':False,'ros_domain_id':os.environ['ROS_DOMAIN_ID'],
                'sessions':[str(s) for s in sessions],'rows_first_session':m['written_messages'],
                'checked':['waits for alignment','stale alignment rejected','automatic start','raw positions unchanged','ID31 p/q/r retained',
                           'NaN retained','target and velocity CSV','five PNG and vector PDF figures','same-flight comparison figures and unchanged points','no rosbag','Ctrl+C automatic save',
                           'navigation child exit automatic save','realignment stays in current session']}
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=90)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for stream in streams:stream.close()
        sender.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    try:result=run()
    except Exception as exc:result={'status':'failed','error':str(exc),'hardware_accessed':False}
    output=ROOT/'log/paper-logging-validation.json';output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(output.read_text())
    raise SystemExit(0 if result['status']=='passed' else 1)
