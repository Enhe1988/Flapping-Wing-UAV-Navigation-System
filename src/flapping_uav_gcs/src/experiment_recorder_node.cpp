#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <spawn.h>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "flapping_uav_gcs/velocity_rotation.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/body_attitude_setpoint.hpp"
#include "flapping_uav_msgs/msg/position_attitude_debug.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "flapping_uav_msgs/msg/body_velocity_setpoint.hpp"
#include "flapping_uav_msgs/msg/heading_alignment.hpp"
#include "flapping_uav_msgs/msg/navigation_status.hpp"
#include "flapping_uav_msgs/msg/position_estimate.hpp"

extern char ** environ;
namespace fs = std::filesystem;
namespace flapping_uav_gcs
{
using Steady = std::chrono::steady_clock;
using Alignment = flapping_uav_msgs::msg::HeadingAlignment;
int64_t steady_ns()
{return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();}
std::string quoted(const std::string & text)
{
  std::ostringstream out;out << '"';
  for (unsigned char c:text) {
    switch(c) {
      case '"':out << "\\\"";break;case '\\':out << "\\\\";break;
      case '\n':out << "\\n";break;case '\r':out << "\\r";break;case '\t':out << "\\t";break;
      default:if(c<32){out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;}
        else {out << c;}
    }
  }
  out << '"';return out.str();
}
std::string csv_text(const std::string & text)
{std::string out="\"";for(char c:text){out+=c;if(c=='"'){out+='"';}}return out+'"';}
void atomic_text(const fs::path & path,const std::string & text)
{
  const auto temporary=path.string()+".tmp";std::ofstream out(temporary);
  out.exceptions(std::ios::failbit|std::ios::badbit);out << text;out.close();fs::rename(temporary,path);
}
struct CsvRow {std::string file,line;};
struct PaperSession
{
  fs::path directory;
  int64_t start_ros{0},start_steady{0},stop_ros{0},stop_steady{0};
  Alignment initial_alignment;
  bool assistance_known{false},assistance_enabled{false};
  std::string flight_mode{"waypoint"};double trajectory_period{0};
  std::string start_trigger{"heading_alignment"};
  std::map<std::string,std::ofstream> files;
  std::map<std::string,uint64_t> counts;
  std::deque<CsvRow> queue;
  std::mutex mutex;std::condition_variable condition;std::thread worker;
  bool closing{false};std::atomic<bool> failed{false};
  std::atomic<uint64_t> received{0},written{0},dropped{0};std::string error;
};
class ExperimentRecorder : public rclcpp::Node
{
public:
  ExperimentRecorder():Node("experiment_recorder_node")
  {
    root_=fs::absolute(declare_parameter<std::string>("root_directory","log/experiments"));
    exporter_=declare_parameter<std::string>("exporter_script","");
    label_=declare_parameter<std::string>("experiment_label","flight");
    config_paths_=declare_parameter<std::vector<std::string>>("config_files",{});
    queue_limit_=declare_parameter<int>("queue_limit",20000);
    export_on_stop_=declare_parameter<bool>("export_on_stop",true);
    start_on_tracking_=declare_parameter<bool>("start_on_trajectory_tracking",true);
    flight_mode_=declare_parameter<std::string>("flight_mode","waypoint");
    if(flight_mode_!="waypoint" && flight_mode_!="line" && flight_mode_!="figure8"){
      throw std::invalid_argument("Invalid logging flight_mode");}
    if(queue_limit_<1){throw std::invalid_argument("queue_limit must be positive");}
    std::ifstream boot("/proc/sys/kernel/random/boot_id");std::getline(boot,host_boot_id_);
    const std::string prefix="t_s,receive_ros_ns,source_ros_ns,";
    headers_={
      {"trajectory.csv",prefix+"x_m,y_m,z_uwb_raw_m,frame_id"},
      {"uwb_filtered.csv",prefix+"x_m,y_m,z_uwb_raw_m,frame_id"},
      {"reference.csv",prefix+"goal_x_m,goal_y_m,goal_active,enabled,inputs_valid,state,vx_map_desired_mps,vy_map_desired_mps,feedback_assumed_zero,flight_mode,trajectory_phase,trajectory_running,trajectory_time_s,trajectory_period_s,trajectory_vx_mps,trajectory_vy_mps"},
      {"control_position.csv",prefix+"x_m,y_m,valid,source,assistance_enabled"},
      {"position_sources.csv",prefix+"selected_x_m,selected_y_m,odom_x_m,odom_y_m,raw_uwb_x_m,raw_uwb_y_m,valid,source,uwb_healthy,flow_valid,raw_uwb_age_ms,reliable_uwb_age_ms,flow_age_ms,innovation_m,uwb_syncs"},
      {"velocity_desired.csv",prefix+"forward_mps,right_mps,valid,frame_id"},
      {"velocity_sent.csv",prefix+"forward_wire_mps,lateral_wire_mps,z_wire_mps,valid,frame_id,command_age_ms,attitude_age_ms,reason"},
      {"attitude_desired.csv",prefix+"pitch_deg,roll_deg,valid,command_age_ms,attitude_age_ms,reason"},
      {"attitude_sent.csv",prefix+"pitch_deg,roll_deg,valid,command_age_ms,attitude_age_ms,reason"},
      {"attitude_pid.csv",prefix+"p_map_x_deg,p_map_y_deg,i_map_x_deg,i_map_y_deg,d_map_x_deg,d_map_y_deg,uwb_vx_est_mps,uwb_vy_est_mps,valid,velocity_estimate_valid,integrating,limited"},
      {"velocity_actual.csv",prefix+"device_time_us,forward_mps,right_mps,velocity_valid,height_up_m,height_valid"},
      {"attitude.csv",prefix+"device_time_us,reset_counter,attitude_valid,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_fc_deg,yaw_map_deg,p_rad_s,q_rad_s,r_rad_s,angular_velocity_valid,yaw_only,message_id"},
      {"alignment.csv",prefix+"device_time_us,host_monotonic_ns,map_to_enu_yaw_rad,quaternion_body_to_ned,valid"},
      {"events.csv",prefix+"event,detail"}};
    using namespace flapping_uav_msgs::msg;
    const auto qos=rclcpp::QoS(200).best_effort();
    add<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position","trajectory.csv",qos,
      [](const auto & m,auto & out){const auto & p=m.pose.pose.position;
        out << p.x << ',' << p.y << ',' << p.z << ',' << csv_text(m.header.frame_id);});
    add<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position_filtered","uwb_filtered.csv",qos,
      [](const auto & m,auto & out){const auto & p=m.pose.pose.position;
        out << p.x << ',' << p.y << ',' << p.z << ',' << csv_text(m.header.frame_id);});
    add<BodyVelocitySetpoint>("/fc/velocity_setpoint_body","velocity_desired.csv",qos,
      [](const auto & m,auto & out){out << m.velocity.x << ',' << m.velocity.y << ',' << m.valid << ',' << csv_text(m.header.frame_id);});
    for(const auto & pair:std::vector<std::pair<std::string,std::string>>{
      {"/fc/attitude_setpoint_body","attitude_desired.csv"},{"/fc/attitude_setpoint_sent","attitude_sent.csv"}}){
      add<BodyAttitudeSetpoint>(pair.first,pair.second,qos,[](const auto & m,auto & out){
        out<<m.pitch_deg<<','<<m.roll_deg<<','<<m.valid<<','<<m.command_age_ms<<','<<m.attitude_age_ms<<','<<csv_text(m.reason);});}
    add<BodyVelocitySetpoint>("/fc/velocity_setpoint_sent","velocity_sent.csv",qos,
      [](const auto & m,auto & out){out << m.velocity.x << ',' << m.velocity.y << ',' << m.velocity.z << ',' << m.valid << ','
        << csv_text(m.header.frame_id) << ',' << m.command_age_ms << ',' << m.attitude_age_ms << ',' << csv_text(m.reason);});
    add<BodyMotionState>("/fc/body_motion","velocity_actual.csv",qos,
      [](const auto & m,auto & out){out << m.sample_time_us << ',' << m.forward_velocity_mps << ',' << m.right_velocity_mps << ','
        << m.velocity_valid << ',' << m.height_up_m << ',' << m.height_valid;});
    add<PositionAttitudeDebug>("/navigation/attitude_pid","attitude_pid.csv",qos,
      [](const auto & m,auto & out){out << m.p_map_deg.x << ',' << m.p_map_deg.y << ','
        << m.i_map_deg.x << ',' << m.i_map_deg.y << ',' << m.d_map_deg.x << ',' << m.d_map_deg.y << ','
        << m.velocity_map_mps.x << ',' << m.velocity_map_mps.y << ',' << m.valid << ','
        << m.velocity_valid << ',' << m.integrating << ',' << m.limited;});
    add<NavigationStatus>("/navigation/status","reference.csv",qos,
      [this](const auto & m,auto & out){out << m.goal.x << ',' << m.goal.y << ',' << m.goal_active << ',' << m.enabled << ','
        << m.inputs_valid << ',' << csv_text(m.state) << ',' << m.velocity_map.x << ',' << m.velocity_map.y << ',' << m.feedback_assumed_zero << ','
        << csv_text(m.flight_mode) << ',' << csv_text(m.trajectory_phase) << ',' << m.trajectory_running << ',' << m.trajectory_time_s << ','
        << m.trajectory_period_s << ',' << m.trajectory_velocity_map.x << ',' << m.trajectory_velocity_map.y;
        auto position=row(m.header);position << m.position_used.x << ',' << m.position_used.y << ',' << m.position_valid << ','
          << csv_text(m.position_source) << ',' << m.optical_flow_fallback_enabled;enqueue("control_position.csv",position.str());
        active_->assistance_known=true;active_->assistance_enabled=m.optical_flow_fallback_enabled;
        active_->flight_mode=m.flight_mode;active_->trajectory_period=m.trajectory_period_s;
        if(!nav_seen_ || m.trajectory_phase!=last_trajectory_phase_){event(m.header,"trajectory",m.flight_mode+": "+m.trajectory_phase);}
        last_trajectory_phase_=m.trajectory_phase;
        if(!nav_seen_ || m.position_source!=last_source_){event(m.header,"position_source",m.position_source);}
        last_source_=m.position_source;
        if(!nav_seen_ || m.state!=last_state_){event(m.header,"navigation",m.state+": "+m.reason);}
        if(!m.trajectory_running && m.goal_active && (!goal_active_ || m.goal.x!=goal_x_ || m.goal.y!=goal_y_)){
          std::ostringstream text;text << std::setprecision(17) << "map XY=(" << m.goal.x << ',' << m.goal.y << ')';
          event(m.header,"goal",text.str());}
        last_state_=m.state;goal_x_=m.goal.x;goal_y_=m.goal.y;goal_active_=m.goal_active;nav_seen_=true;},
      [this](const NavigationStatus & m){maybe_start_navigation(m);});
    add<PositionEstimate>("/localization/position","position_sources.csv",qos,
      [](const auto & m,auto & out){out << m.position.x << ',' << m.position.y << ','
        << m.optical_odometry_position.x << ',' << m.optical_odometry_position.y << ','
        << m.raw_uwb_position.x << ',' << m.raw_uwb_position.y << ',' << m.valid << ',' << csv_text(m.source) << ','
        << m.uwb_healthy << ',' << m.flow_valid << ',' << m.raw_uwb_age_ms << ',' << m.reliable_uwb_age_ms << ','
        << m.flow_age_ms << ',' << m.innovation_m << ',' << m.uwb_syncs;});
    add<AttitudeState>("/fc/attitude","attitude.csv",qos,
      [this](const auto & m,auto & out){
        const auto & o=m.orientation;QuaternionWxyz q{{o.w,o.x,o.y,o.z}};
        double roll=NAN,pitch=NAN,yaw=NAN,map_yaw=NAN;
        if(m.yaw_only){yaw=m.valid?m.yaw_rad:NAN;}
        else if(m.valid && normalize_attitude(q)){
          if(!alignment_.quaternion_body_to_ned){for(size_t i=1;i<4;++i){q[i]=-q[i];}}
          const double w=q[0],x=q[1],y=q[2],z=q[3];
          roll=std::atan2(2*(w*x+y*z),1-2*(x*x+y*y));
          pitch=std::asin(std::max(-1.,std::min(1.,2*(w*y-z*x))));
          yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
        }
        if(std::isfinite(yaw)){
          const double elapsed_host=(steady_ns()-alignment_.host_monotonic_ns)/1e6;
          const double elapsed_device=(static_cast<long double>(m.sample_time_us)-alignment_.device_time_us)/1000.;
          if(alignment_.valid && m.sample_time_us>=static_cast<uint64_t>(alignment_.device_time_us) &&
            std::abs(elapsed_host-elapsed_device)<=alignment_.max_clock_skew_ms){
            const double angle=std::acos(-1.)/2-yaw-alignment_.map_to_enu_yaw_rad;
            map_yaw=std::atan2(std::sin(angle),std::cos(angle));}
        }
        const double deg=180/std::acos(-1.);
        out << m.sample_time_us << ',' << int(m.reset_counter) << ',' << m.valid << ',' << o.w << ',' << o.x << ',' << o.y << ',' << o.z << ','
          << roll*deg << ',' << pitch*deg << ',' << yaw*deg << ',' << map_yaw*deg << ','
          << m.angular_velocity.x << ',' << m.angular_velocity.y << ',' << m.angular_velocity.z << ',' << m.angular_velocity_valid
          << ',' << m.yaw_only << ',' << m.message_id;
      });
    alignment_sub_=create_subscription<Alignment>("/navigation/heading_alignment",rclcpp::QoS(1).reliable().transient_local(),
      [this](Alignment::ConstSharedPtr m){
        const double age_ms=(now().nanoseconds()-rclcpp::Time(m->header.stamp).nanoseconds())/1e6;
        const bool fresh=m->valid && m->header.frame_id=="map" && age_ms>=-100 && age_ms<=5000 &&
          m->device_time_us>0 && m->host_monotonic_ns>0 && m->host_monotonic_ns<=steady_ns() &&
          m->host_boot_id==host_boot_id_ && std::isfinite(m->map_to_enu_yaw_rad);
        if(!active_ && !fresh){
          if(!m->valid && m->header.frame_id=="map"){
            alignment_=*m;auto_start_armed_=false;state_="waiting_for_alignment";publish_status();}
          return;
        }
        const bool new_alignment=m->host_monotonic_ns!=alignment_.host_monotonic_ns;
        alignment_=*m;
        if(!active_ && fresh && new_alignment){
          auto_start_armed_=true;
          if(wait_for_tracking()){
            state_="waiting_for_trajectory";publish_status();
            if(navigation_seen_){maybe_start_navigation(latest_navigation_);}
          }else{start(rclcpp::Time(m->header.stamp).nanoseconds(),"heading_alignment");}
        }else if(active_){record_alignment(*m);}
      });
    status_=create_publisher<std_msgs::msg::String>("/experiment/status",rclcpp::QoS(1).reliable().transient_local());
    // Optional compatibility services; ordinary flights never need these.
    start_=create_service<std_srvs::srv::Trigger>("/experiment/start",[this](
      std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){
        try{
          if(!active_ && alignment_.valid){
            auto_start_armed_=true;
            if(wait_for_tracking()){if(navigation_seen_){maybe_start_navigation(latest_navigation_);}}
            else{start(0,"manual_start");}
          }
          r->success=bool(active_);r->message=r->success?last_directory_.string():
            (alignment_.valid?"Waiting for valid trajectory TRACKING after actual arrival at start point":"Align via /navigation/start_here first");}
        catch(const std::exception & e){r->success=false;r->message=e.what();}});
    stop_=create_service<std_srvs::srv::Trigger>("/experiment/stop",[this](
      std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){
        try{r->success=finish() && last_error_.empty();r->message=last_directory_.string();if(!last_error_.empty()){r->message+="; "+last_error_;}}
        catch(const std::exception & e){r->success=false;r->message=e.what();}});
    for(const std::string name:{"position_controller_node","mavlink_bridge_node","uwb_position_node","fusion_odometry_node"}){
      parameter_clients_[name]=std::make_shared<rclcpp::AsyncParametersClient>(this,name);}
    timer_=create_wall_timer(std::chrono::seconds(1),[this](){snapshot_parameters();publish_status();});
    publish_status();RCLCPP_INFO(get_logger(),"Paper CSV logger: %s; no rosbag",
      wait_for_tracking()?"align first, then wait for trajectory TRACKING (no approach recording)":"start on fresh heading alignment");
  }
  ~ExperimentRecorder() override
  {try{finish();}catch(const std::exception & e){RCLCPP_ERROR(get_logger(),"Close recording: %s",e.what());}}
  bool finish()
  {
    if(!active_){return false;}
    auto_start_armed_=false;
    auto session=std::move(active_);
    {std::lock_guard<std::mutex> lock(session->mutex);session->closing=true;}
    session->condition.notify_one();session->worker.join();
    session->stop_ros=now().nanoseconds();session->stop_steady=steady_ns();
    session->files.clear();last_error_=session->error;
    last_received_=session->received;last_written_=session->written;last_dropped_=session->dropped;
    write_manifest(*session,session->failed?"recording_error":"recorded");
    if(export_on_stop_ && !exporter_.empty()){
      state_="exporting";publish_status();const int code=export_report(session->directory);
      if(code!=0){last_error_+="Figure export failed (exit "+std::to_string(code)+"); raw CSV retained";}}
    state_=last_error_.empty()?"finished":"error";publish_status();
    RCLCPP_INFO(get_logger(),"Paper experiment saved: %s; rows=%llu; %s",session->directory.c_str(),
      static_cast<unsigned long long>(session->written.load()),last_error_.c_str());return true;
  }
private:
  bool wait_for_tracking() const {return start_on_tracking_ && flight_mode_!="waypoint";}
  void maybe_start_navigation(const flapping_uav_msgs::msg::NavigationStatus & m)
  {
    latest_navigation_=m;navigation_seen_=true;
    if(active_ || !auto_start_armed_ || !wait_for_tracking() || !alignment_.valid){return;}
    const int64_t stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    const double age_ms=(now().nanoseconds()-stamp)/1e6;
    if(m.flight_mode!=flight_mode_ || m.header.frame_id!="map" || !m.enabled || !m.inputs_valid ||
      !m.goal_active || !m.trajectory_running || m.trajectory_phase!="TRACKING" ||
      stamp<rclcpp::Time(alignment_.header.stamp).nanoseconds() || age_ms<0 || age_ms>500){return;}
    start(stamp,"trajectory_tracking");
  }
  void record_alignment(const Alignment & m)
  {
    auto out=row(m.header);out << m.device_time_us << ',' << m.host_monotonic_ns << ',' << m.map_to_enu_yaw_rad << ','
      << m.quaternion_body_to_ned << ',' << m.valid;enqueue("alignment.csv",out.str());
    event(m.header,m.valid?"heading_aligned":"heading_invalid",m.reason);
  }
  std::ostringstream row(const std_msgs::msg::Header & header)
  {
    std::ostringstream out;out << std::setprecision(17) << (steady_ns()-active_->start_steady)/1e9 << ','
      << now().nanoseconds() << ',' << rclcpp::Time(header.stamp).nanoseconds() << ',';return out;
  }
  template<typename T,typename F> void add(const std::string & topic,const std::string & file,const rclcpp::QoS & qos,F formatter,
    std::function<void(const T &)> before={})
  {
    subscriptions_.push_back(create_subscription<T>(topic,qos,[this,file,formatter,before](typename T::ConstSharedPtr m){
      if(before){before(*m);}
      if(!active_ || active_->failed || rclcpp::Time(m->header.stamp).nanoseconds()<active_->start_ros){return;}
      auto out=row(m->header);formatter(*m,out);enqueue(file,out.str());}));
  }
  void enqueue(const std::string & file,const std::string & line)
  {
    if(!active_ || active_->failed){return;}auto & s=*active_;++s.received;
    {std::lock_guard<std::mutex> lock(s.mutex);
      if(s.queue.size()>=static_cast<size_t>(queue_limit_)){++s.dropped;return;}
      s.queue.push_back({file,line});}
    s.condition.notify_one();
  }
  void event(const std_msgs::msg::Header & header,const std::string & kind,const std::string & detail)
  {auto out=row(header);out << csv_text(kind) << ',' << csv_text(detail);enqueue("events.csv",out.str());}
  bool start(int64_t trigger_ros_ns=0,const std::string & trigger="heading_alignment")
  {
    if(active_){return false;}fs::create_directories(root_);
    const auto wall=std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());std::tm tm{};localtime_r(&wall,&tm);
    std::string safe;for(unsigned char c:label_){safe+=(std::isalnum(c)||c=='-'||c=='_')?char(c):'_';}
    std::ostringstream id;id << std::put_time(&tm,"%Y%m%d_%H%M%S") << '_' << (safe.empty()?"flight":safe) << '_' << steady_ns();
    auto s=std::make_unique<PaperSession>();s->directory=root_/id.str();
    fs::create_directories(s->directory/"config");fs::create_directories(s->directory/"parameters");
    const int64_t current_ros=now().nanoseconds(),current_steady=steady_ns();
    s->start_ros=trigger_ros_ns>0?trigger_ros_ns:current_ros;
    s->start_steady=current_steady-(current_ros-s->start_ros);s->initial_alignment=alignment_;
    s->start_trigger=trigger;s->flight_mode=flight_mode_;
    for(size_t i=0;i<config_paths_.size();++i){fs::path source(config_paths_[i]);
      if(fs::is_regular_file(source)){fs::copy_file(source,s->directory/"config"/(std::to_string(i)+"_"+source.filename().string()));}}
    for(const auto & item:headers_){auto & file=s->files[item.first];file.exceptions(std::ios::failbit|std::ios::badbit);
      file.open(s->directory/item.first);file << "\xef\xbb\xbf" << item.second << '\n';}
    write_manifest(*s,"recording");parameter_requested_.clear();last_error_.clear();last_directory_=s->directory;
    nav_seen_=goal_active_=false;auto * ptr=s.get();
    ptr->worker=std::thread([ptr](){try{while(true){CsvRow item;
      {std::unique_lock<std::mutex> lock(ptr->mutex);ptr->condition.wait(lock,[ptr](){return ptr->closing||!ptr->queue.empty();});
        if(ptr->queue.empty()&&ptr->closing){break;}item=std::move(ptr->queue.front());ptr->queue.pop_front();}
      ptr->files.at(item.file) << item.line << '\n';++ptr->counts[item.file];const auto n=++ptr->written;
      if(n%256==0){for(auto & file:ptr->files){file.second.flush();}}
    }for(auto & file:ptr->files){file.second.flush();}}
      catch(const std::exception & e){ptr->error=e.what();ptr->failed=true;}});
    active_=std::move(s);state_="recording";
    // Keep the heading reference even though it was captured before TRACKING.
    // No position, attitude, command or reference rows from APPROACH are replayed.
    record_alignment(alignment_);
    std_msgs::msg::Header header;header.stamp=rclcpp::Time(active_->start_ros);header.frame_id="map";
    event(header,"recording_started",trigger);publish_status();
    RCLCPP_INFO(get_logger(),"Paper recording started (%s): %s",trigger.c_str(),last_directory_.c_str());return true;
  }
  void snapshot_parameters()
  {
    if(!active_){return;}
    for(auto & item:parameter_clients_){
      if(parameter_requested_[item.first] || !item.second->service_is_ready()){continue;}
      parameter_requested_[item.first]=true;const auto target=item.first;const auto directory=active_->directory;const auto client=item.second;
      client->list_parameters({},100,[this,client,target,directory](auto names_future){try{
        client->get_parameters(names_future.get().names,[this,target,directory](auto values_future){try{
          std::ostringstream out;out << '{';bool first=true;for(const auto & p:values_future.get()){
            if(!first){out << ',';}first=false;out << quoted(p.get_name()) << ":{\"type\":" << quoted(p.get_type_name())
              << ",\"value_text\":" << quoted(p.value_to_string()) << '}';}
          out << "}\n";atomic_text(directory/"parameters"/(target+".json"),out.str());
        }catch(const std::exception & e){RCLCPP_WARN(get_logger(),"Parameter snapshot: %s",e.what());}});
      }catch(const std::exception & e){RCLCPP_WARN(get_logger(),"Parameter list: %s",e.what());}});
    }
  }
  void write_manifest(const PaperSession & s,const std::string & state)
  {
    std::ostringstream out;out << std::setprecision(17) << "{\"schema_version\":2,\"format\":\"paper_csv_v1\",\"label\":" << quoted(label_)
      << ",\"state\":" << quoted(state) << ",\"start_ros_ns\":" << s.start_ros << ",\"start_steady_ns\":" << s.start_steady
      << ",\"stop_ros_ns\":" << s.stop_ros << ",\"stop_steady_ns\":" << s.stop_steady << ",\"received_messages\":" << s.received
      << ",\"written_messages\":" << s.written << ",\"dropped_messages\":" << s.dropped << ",\"recording_error\":" << quoted(s.error)
      << ",\"host_boot_id\":" << quoted(host_boot_id_) << ",\"initial_map_to_enu_yaw_rad\":" << s.initial_alignment.map_to_enu_yaw_rad
      << ",\"quaternion_body_to_ned\":" << (s.initial_alignment.quaternion_body_to_ned?"true":"false")
      << ",\"flow_assistance_enabled\":" << (s.assistance_known?(s.assistance_enabled?"true":"false"):"null")
      << ",\"flight_mode\":" << quoted(s.flight_mode) << ",\"trajectory_period_s\":" << s.trajectory_period
      << ",\"recording_start_trigger\":" << quoted(s.start_trigger)
      << ",\"heading_alignment_ros_ns\":" << rclcpp::Time(s.initial_alignment.header.stamp).nanoseconds()
      << ",\"files\":{";bool first=true;
    for(const auto & item:headers_){if(!first){out << ',';}first=false;const auto count=s.counts.find(item.first);
      out << quoted(item.first) << ":{\"rows\":" << (count==s.counts.end()?0:count->second) << '}';}
    out << "},\"notes\":\"Unfiltered source samples; map XY in metres, forward/right velocity in m/s, Euler angles in degrees, reported body angular rates p/q/r in rad/s. No rosbag.\"}\n";
    atomic_text(s.directory/"session.json",out.str());
  }
  int export_report(const fs::path & directory)
  {
    std::vector<std::string> args{"python3",exporter_,"export",directory.string()};std::vector<char *> argv;
    for(auto & arg:args){argv.push_back(arg.data());}argv.push_back(nullptr);pid_t pid{};
    const int result=posix_spawnp(&pid,"python3",nullptr,nullptr,argv.data(),environ);if(result){return result;}
    int status{};while(waitpid(pid,&status,0)<0){if(errno!=EINTR){return 127;}}return WIFEXITED(status)?WEXITSTATUS(status):128;
  }
  void publish_status()
  {
    if(!rclcpp::ok()){return;}std_msgs::msg::String message;std::ostringstream out;
    out << "{\"format\":\"paper_csv_v1\",\"state\":" << quoted(state_) << ",\"directory\":" << quoted(last_directory_.string())
      << ",\"received\":" << (active_?active_->received.load():last_received_) << ",\"written\":" << (active_?active_->written.load():last_written_)
      << ",\"dropped\":" << (active_?active_->dropped.load():last_dropped_) << ",\"error\":"
      << quoted(active_&&active_->failed?"CSV writer failed; raw completed rows retained":last_error_) << '}';
    message.data=out.str();status_->publish(message);
  }
  fs::path root_,last_directory_;std::string exporter_,label_,host_boot_id_,state_{"waiting_for_alignment"},last_error_,last_state_,last_source_,last_trajectory_phase_;
  bool export_on_stop_,start_on_tracking_,auto_start_armed_{false},navigation_seen_{false},nav_seen_{false},goal_active_{false};
  int queue_limit_;double goal_x_{0},goal_y_{0};Alignment alignment_;
  std::string flight_mode_;flapping_uav_msgs::msg::NavigationStatus latest_navigation_;
  uint64_t last_received_{0},last_written_{0},last_dropped_{0};
  std::vector<std::string> config_paths_;std::map<std::string,std::string> headers_;
  std::unique_ptr<PaperSession> active_;std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string,std::shared_ptr<rclcpp::AsyncParametersClient>> parameter_clients_;std::map<std::string,bool> parameter_requested_;
  rclcpp::Subscription<Alignment>::SharedPtr alignment_sub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_,stop_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try{auto node=std::make_shared<flapping_uav_gcs::ExperimentRecorder>();rclcpp::spin(node);node->finish();node.reset();}
  catch(const std::exception & e){RCLCPP_FATAL(rclcpp::get_logger("experiment_recorder_node"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
