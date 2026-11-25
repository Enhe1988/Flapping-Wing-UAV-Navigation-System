#!/usr/bin/env python3
"""Check automatic logging gates with synthetic ROS data, no FC or serial access."""
import csv,json,os,signal,subprocess,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def require(ok,message):
    if not ok:raise RuntimeError(message)

def main():
    os.environ['ROS_DOMAIN_ID']='232';os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from flapping_uav_msgs.msg import HeadingAlignment,NavigationStatus
    rclpy.init();node=rclpy.create_node('tracking_log_probe')
    alignment=node.create_publisher(HeadingAlignment,'/navigation/heading_alignment',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    navigation=node.create_publisher(NavigationStatus,'/navigation/status',10)
    pose=node.create_publisher(PoseWithCovarianceStamped,'/uwb/position',10)
    root=ROOT/'log/experiments_demo'/('tracking_logging_'+str(time.monotonic_ns()));root.mkdir(parents=True)
    results=[]
    try:
        for mode,gated in [('figure8',True),('waypoint',True),('line',False)]:
            invalid=HeadingAlignment();invalid.header.stamp=node.get_clock().now().to_msg();invalid.header.frame_id='map'
            alignment.publish(invalid)
            folder=root/(mode+('_gated' if gated else '_alignment'));folder.mkdir()
            with (folder/'recorder.log').open('w') as output:
                process=subprocess.Popen(['ros2','run','flapping_uav_gcs','experiment_recorder_node','--ros-args',
                    '-p','root_directory:='+str(folder),'-p','flight_mode:='+mode,
                    '-p','start_on_trajectory_tracking:='+str(gated).lower(),'-p','export_on_stop:=false',
                    '-p','experiment_label:=synthetic_tracking_logger'],stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                end=time.monotonic()+8
                while time.monotonic()<end and navigation.get_subscription_count()==0:
                    rclpy.spin_once(node,timeout_sec=.05)
                require(navigation.get_subscription_count()>0,'Recorder did not subscribe')
                phase='WAITING_START';valid=True
                def feed(seconds):
                    end=time.monotonic()+seconds
                    while time.monotonic()<end:
                        stamp=node.get_clock().now().to_msg()
                        n=NavigationStatus();n.header.stamp=stamp;n.header.frame_id='map';n.enabled=True
                        n.inputs_valid=valid;n.goal_active=True;n.state='ACTIVE' if valid else 'INPUT_HOLD'
                        n.flight_mode=mode;n.trajectory_phase=phase;n.trajectory_running=phase in ('APPROACH','TRACKING')
                        n.trajectory_period_s=80.;n.position_valid=valid;n.position_source='UWB'
                        n.position_used.x=2.;n.position_used.y=-3.;n.goal.x=2.;n.goal.y=-3.;navigation.publish(n)
                        p=PoseWithCovarianceStamped();p.header=n.header
                        p.pose.pose.position.x=2. if phase=='TRACKING' else 99.;p.pose.pose.position.y=-3.;pose.publish(p)
                        rclpy.spin_once(node,timeout_sec=.02)
                feed(.2);require(not list(folder.glob('*/session.json')),'Logging started before alignment')
                a=HeadingAlignment();a.header.stamp=node.get_clock().now().to_msg();a.header.frame_id='map';a.valid=True
                a.device_time_us=1000000;a.host_monotonic_ns=time.monotonic_ns()
                a.host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip();a.quaternion_body_to_ned=True
                a.max_clock_skew_ms=2000.;alignment.publish(a);feed(.2)
                if mode!='waypoint' and gated:
                    require(not list(folder.glob('*/session.json')),'Alignment started trajectory log')
                    phase='APPROACH';feed(.3)
                    require(not list(folder.glob('*/session.json')),'Approach started trajectory log')
                    phase='TRACKING';valid=False;feed(.2)
                    require(not list(folder.glob('*/session.json')),'Invalid tracking started log')
                    valid=True
                else:require(len(list(folder.glob('*/session.json')))==1,'Waypoint/legacy alignment start missing')
                phase='TRACKING';feed(.4)
                files=list(folder.glob('*/session.json'));require(len(files)==1,'Tracking did not create one log')
                session=files[0].parent;m=json.loads(files[0].read_text())
                if mode!='waypoint' and gated:
                    require(m['recording_start_trigger']=='trajectory_tracking','Wrong start trigger')
                    old=PoseWithCovarianceStamped();old.header.frame_id='map'
                    sec,nanosec=divmod(m['start_ros_ns']-1,1000000000)
                    old.header.stamp.sec=sec;old.header.stamp.nanosec=nanosec
                    old.pose.pose.position.x=333.;pose.publish(old);feed(.3)
                os.killpg(process.pid,signal.SIGINT);process.wait(timeout=12)
                require(process.returncode==0,'Recorder shutdown failed')
                with (session/'reference.csv').open(encoding='utf-8-sig') as stream:rows=list(csv.DictReader(stream))
                with (session/'trajectory.csv').open(encoding='utf-8-sig') as stream:points=list(csv.DictReader(stream))
                require(rows and points,'Empty log')
                if mode!='waypoint' and gated:
                    require(all(r['trajectory_phase']=='TRACKING' for r in rows),'Preparation references leaked')
                    require(all(float(p['x_m'])==2. for p in points),'Preparation or queued old pose leaked')
                    require(all(int(r['source_ros_ns'])>=m['start_ros_ns'] for r in rows+points),'Pre-trigger samples leaked')
                m=json.loads(files[0].read_text());require(m['state']=='recorded' and m['files']['alignment.csv']['rows']==1,'Alignment seed/finalization missing')
                require(not m['dropped_messages'] and not m['recording_error'],'Writer lost data')
                results.append({'flight_mode':mode,'gated':gated,'trigger':m['recording_start_trigger'],'session':str(session)})
            finally:
                if process.poll() is None:os.killpg(process.pid,signal.SIGINT);process.wait(timeout=12)
                end=time.monotonic()+.3
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    finally:node.destroy_node();rclpy.shutdown()
    result={'status':'passed','hardware_accessed':False,'data_kind':'SYNTHETIC LOGGER CHECK; NOT FLIGHT DATA','scenarios':results}
    (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
