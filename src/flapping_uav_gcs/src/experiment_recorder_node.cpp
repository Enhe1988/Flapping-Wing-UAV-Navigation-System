#include <atomic>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <spawn.h>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <vector>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "nlink_parser2/msg/linktrack_anchorframe0.hpp"
#include "flapping_uav_msgs/msg/attitude_state.hpp"
#include "flapping_uav_msgs/msg/body_motion_state.hpp"
#include "flapping_uav_msgs/msg/body_velocity_setpoint.hpp"
#include "flapping_uav_msgs/msg/heading_alignment.hpp"
#include "flapping_uav_msgs/msg/mavlink_frame.hpp"
#include "flapping_uav_msgs/msg/navigation_status.hpp"
#include "flapping_uav_msgs/msg/rc_override.hpp"
#include "flapping_uav_msgs/msg/uwb_status.hpp"
#include "flapping_uav_msgs/msg/fusion_status.hpp"
#include "nav_msgs/msg/odometry.hpp"

extern char ** environ;
namespace fs = std::filesystem;
namespace flapping_uav_gcs
{
using Steady = std::chrono::steady_clock;
int64_t steady_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();}
std::string quoted(const std::string & s)
{
  std::ostringstream out; out << '"';
  for (unsigned char c : s) {
    switch (c) {
      case '"': out << "\\\""; break; case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break; case '\r': out << "\\r"; break; case '\t': out << "\\t"; break;
      default: if (c < 32) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;}
        else {out << c;}
    }
  }
  out << '"'; return out.str();
}
void atomic_text(const fs::path & path, const std::string & text)
{
  const auto temporary = path.string()+".tmp";
  std::ofstream out(temporary); out.exceptions(std::ios::failbit | std::ios::badbit);
  out << text; out.close(); fs::rename(temporary, path);
}
struct RecordedItem
{
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
  int64_t steady;
};
struct RecordingSession
{
  fs::path directory;
  int64_t start_ros{0}, start_steady{0}, stop_ros{0}, stop_steady{0};
  std::unique_ptr<rosbag2_cpp::Writer> bag;
  std::ofstream timeline;
  std::mutex mutex; std::condition_variable condition;
  std::deque<RecordedItem> queue;
  std::thread worker;
  bool closing{false};
  std::atomic<uint64_t> received{0}, written{0}, dropped{0};
  std::atomic<bool> failed{false};
  std::string error;
  std::map<std::string, uint64_t> counts;
};
class ExperimentRecorder : public rclcpp::Node
{
public:
  ExperimentRecorder() : Node("experiment_recorder_node")
  {
    root_ = fs::absolute(declare_parameter<std::string>("root_directory", "log/experiments"));
    exporter_ = declare_parameter<std::string>("exporter_script", "");
    export_on_stop_ = declare_parameter<bool>("export_on_stop", true);
    label_ = declare_parameter<std::string>("experiment_label", "flight");
    config_paths_ = declare_parameter<std::vector<std::string>>("config_files", {});
    queue_limit_ = declare_parameter<int>("queue_limit", 20000);
    if (queue_limit_ < 1) {throw std::invalid_argument("queue_limit must be positive");}
    const bool auto_start = declare_parameter<bool>("auto_start", true);
    const auto live = rclcpp::QoS(500).best_effort();
    const auto retained = rclcpp::QoS(100).reliable().transient_local();
    using namespace flapping_uav_msgs::msg;
    add<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/position", "geometry_msgs/msg/PoseWithCovarianceStamped", live);
    add<UwbStatus>("/uwb/status", "flapping_uav_msgs/msg/UwbStatus", live);
    add<nlink_parser2::msg::LinktrackAnchorframe0>("/nlink_linktrack_anchorframe0", "nlink_parser2/msg/LinktrackAnchorframe0", live);
    add<nav_msgs::msg::Odometry>("/fusion/odometry", "nav_msgs/msg/Odometry", live);
    add<FusionStatus>("/fusion/status", "flapping_uav_msgs/msg/FusionStatus", live);
    add<AttitudeState>("/fc/attitude", "flapping_uav_msgs/msg/AttitudeState", live);
    add<BodyMotionState>("/fc/body_motion", "flapping_uav_msgs/msg/BodyMotionState", live);
    add<BodyVelocitySetpoint>("/fc/velocity_setpoint_body", "flapping_uav_msgs/msg/BodyVelocitySetpoint", live);
    add<BodyVelocitySetpoint>("/fc/velocity_setpoint_sent", "flapping_uav_msgs/msg/BodyVelocitySetpoint", live);
    add<MavlinkFrame>("/fc/mavlink_rx", "flapping_uav_msgs/msg/MavlinkFrame", live);
    add<NavigationStatus>("/navigation/status", "flapping_uav_msgs/msg/NavigationStatus", live);
    add<geometry_msgs::msg::TwistStamped>("/navigation/velocity_setpoint_map", "geometry_msgs/msg/TwistStamped", live);
    add<HeadingAlignment>("/navigation/heading_alignment", "flapping_uav_msgs/msg/HeadingAlignment", retained);
    add<geometry_msgs::msg::PoseStamped>("/navigation/goal", "geometry_msgs/msg/PoseStamped", live);
    add<geometry_msgs::msg::PoseStamped>("/goal_pose", "geometry_msgs/msg/PoseStamped", live);
    add<RcOverride>("/rc/override", "flapping_uav_msgs/msg/RcOverride", live);
    add<sensor_msgs::msg::Joy>("/joy", "sensor_msgs/msg/Joy", live);
    add<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", live);
    add<rcl_interfaces::msg::Log>("/rosout", "rcl_interfaces/msg/Log", retained);
    add<rcl_interfaces::msg::ParameterEvent>("/parameter_events", "rcl_interfaces/msg/ParameterEvent", live);
    add<std_msgs::msg::String>("/experiment/mark", "std_msgs/msg/String", live);
    status_ = create_publisher<std_msgs::msg::String>(
      "/experiment/status", rclcpp::QoS(1).reliable().transient_local());
    start_ = create_service<std_srvs::srv::Trigger>("/experiment/start", [this](
      std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
        try {r->success = active_ || start(); r->message = active_ ? active_->directory.string() : last_directory_.string();}
        catch (const std::exception & e) {r->success = false; r->message = e.what();}
      });
    stop_ = create_service<std_srvs::srv::Trigger>("/experiment/stop", [this](
      std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
        try {r->success = finish(); r->message = last_directory_.string();
          if (!last_error_.empty()) {r->success = false; r->message += "; " + last_error_;}}
        catch (const std::exception & e) {r->success = false; r->message = e.what();}
      });
    for (const std::string name : {"position_controller_node", "mavlink_bridge_node", "manual_control_node",
      "uwb_position_node", "scene_visualizer_node", "fusion_odometry_node"}) {
      parameter_clients_[name] = std::make_shared<rclcpp::AsyncParametersClient>(this, name);
    }
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {snapshot_parameters(); publish_status();});
    if (auto_start) {start();}
  }
  ~ExperimentRecorder() override
  {
    try {finish();} catch (const std::exception & e) {RCLCPP_ERROR(get_logger(), "Close recording: %s", e.what());}
  }
  bool finish()
  {
    if (!active_) {return false;}
    auto session = std::move(active_);
    {std::lock_guard<std::mutex> lock(session->mutex); session->closing = true;}
    session->condition.notify_one(); session->worker.join();
    session->stop_ros = now().nanoseconds(); session->stop_steady = steady_ns();
    session->bag.reset(); session->timeline.close();
    last_directory_ = session->directory;
    last_error_ = session->error;
    last_received_ = session->received; last_written_ = session->written; last_dropped_ = session->dropped;
    write_manifest(*session, session->failed ? "recording_error" : "recorded");
    if (export_on_stop_ && !exporter_.empty()) {
      state_ = "exporting"; publish_status();
      const int code = export_report(session->directory);
      if (code != 0) {last_error_ += " report export failed (exit " + std::to_string(code) + "); raw data retained";}
    }
    state_ = last_error_.empty() ? "finished" : "error";
    RCLCPP_INFO(get_logger(), "Experiment closed: %s; messages=%llu, dropped=%llu; %s",
      session->directory.c_str(), static_cast<unsigned long long>(session->written.load()),
      static_cast<unsigned long long>(session->dropped.load()), last_error_.c_str());
    publish_status(); return true;
  }
private:
  template<typename T> void add(const std::string & topic, const std::string & type, const rclcpp::QoS & qos)
  {
    topics_.push_back({topic, type, "cdr", ""});
    subscriptions_.push_back(create_subscription<T>(topic, qos,
      [this, topic](typename T::ConstSharedPtr m) {
        const bool keep_alignment = topic == "/navigation/heading_alignment";
        if (!active_ && !keep_alignment) {return;}
        auto bytes = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<T> serializer; serializer.serialize_message(m.get(), bytes.get());
        auto data = std::shared_ptr<rcutils_uint8_array_t>(bytes, &bytes->get_rcl_serialized_message());
        if (keep_alignment) {latest_alignment_ = data;}
        if (!active_ || active_->failed) {return;}
        auto & s = *active_; ++s.received;
        auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        message->serialized_data = data;
        message->topic_name = topic; message->time_stamp = now().nanoseconds();
        const int64_t monotonic = steady_ns();
        {std::lock_guard<std::mutex> lock(s.mutex);
          if (s.queue.size() >= static_cast<size_t>(queue_limit_)) {
            ++s.dropped; RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Experiment log queue full; data dropped (control is independent)"); return;
          }
          s.queue.push_back({message, monotonic});}
        s.condition.notify_one();
      }));
  }
  bool start()
  {
    if (active_) {return false;}
    fs::create_directories(root_);
    const auto wall = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(wall); std::tm tm{}; localtime_r(&time, &tm);
    std::ostringstream id; id << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string safe_label;
    for (unsigned char c : label_) {safe_label += (std::isalnum(c) || c == '-' || c == '_') ? char(c) : '_';}
    if (safe_label.empty()) {safe_label = "flight";}
    id << '_' << safe_label << '_' << steady_ns();
    auto session = std::make_unique<RecordingSession>(); session->directory = root_ / id.str();
    fs::create_directories(session->directory / "config"); fs::create_directories(session->directory / "parameters");
    session->start_ros = now().nanoseconds(); session->start_steady = steady_ns();
    for (size_t i = 0; i < config_paths_.size(); ++i) {
      fs::path source(config_paths_[i]);
      if (fs::is_regular_file(source)) {fs::copy_file(source, session->directory / "config" /
          (std::to_string(i)+"_"+source.filename().string()));}
    }
    session->bag = std::make_unique<rosbag2_cpp::Writer>(std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
    rosbag2_cpp::StorageOptions options; options.uri = (session->directory / "bag").string(); options.storage_id = "sqlite3";
    session->bag->open(options, {"cdr", "cdr"});
    for (const auto & topic : topics_) {session->bag->create_topic(topic);}
    session->timeline.open(session->directory / "timeline.csv");
    session->timeline.exceptions(std::ios::failbit | std::ios::badbit);
    session->timeline << "message_id,topic,receive_ros_ns,receive_steady_ns\n";
    write_manifest(*session, "recording");
    parameter_requested_.clear(); last_error_.clear(); last_directory_ = session->directory;
    auto * ptr = session.get();
    session->worker = std::thread([ptr]() {
      try {
        while (true) {
          RecordedItem item;
          {std::unique_lock<std::mutex> lock(ptr->mutex);
            ptr->condition.wait(lock, [ptr]() {return ptr->closing || !ptr->queue.empty();});
            if (ptr->queue.empty() && ptr->closing) {break;}
            item = std::move(ptr->queue.front()); ptr->queue.pop_front();}
          ptr->bag->write(item.message); const auto index = ++ptr->written;
          ptr->timeline << index << ',' << item.message->topic_name << ',' << item.message->time_stamp << ',' << item.steady << '\n';
          ++ptr->counts[item.message->topic_name];
          if (index % 100 == 0) {ptr->timeline.flush();}
        }
        ptr->timeline.flush();
      } catch (const std::exception & e) {
        ptr->error = e.what(); ptr->failed = true;
      }
    });
    active_ = std::move(session); state_ = "recording";
    // A new experiment can begin without another navigation calibration. Carry
    // the latest alignment into its bag, retaining its ORIGINAL source stamp
    // and boot reference. It is metadata, not a new alignment measurement.
    if (latest_alignment_) {
      auto m = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      m->topic_name = "/navigation/heading_alignment"; m->time_stamp = now().nanoseconds();
      m->serialized_data = latest_alignment_;
      {std::lock_guard<std::mutex> lock(active_->mutex);
        active_->queue.push_back({m, steady_ns()}); ++active_->received;}
      active_->condition.notify_one();
    }
    RCLCPP_INFO(get_logger(), "Recording experiment: %s", active_->directory.c_str()); publish_status(); return true;
  }
  void snapshot_parameters()
  {
    if (!active_) {return;}
    for (auto & item : parameter_clients_) {
      if (parameter_requested_[item.first] || !item.second->service_is_ready()) {continue;}
      parameter_requested_[item.first] = true;
      const auto target = item.first; const auto directory = active_->directory; const auto client = item.second;
      client->list_parameters({}, 100, [this, client, target, directory](auto list_future) {
        try {
          const auto names = list_future.get().names;
          client->get_parameters(names, [this, target, directory](auto values_future) {
            try {
              std::ostringstream out; out << "{"; bool first = true;
              for (const auto & p : values_future.get()) {
                if (!first) {out << ',';} first = false;
                out << quoted(p.get_name()) << ":{\"type\":" << quoted(p.get_type_name())
                    << ",\"value_text\":" << quoted(p.value_to_string()) << '}';
              }
              out << "}\n"; atomic_text(directory / "parameters" / (target+".json"), out.str());
            } catch (const std::exception & e) {RCLCPP_WARN(get_logger(), "Parameter snapshot: %s", e.what());}
          });
        } catch (const std::exception & e) {RCLCPP_WARN(get_logger(), "Parameter list: %s", e.what());}
      });
    }
  }
  void write_manifest(const RecordingSession & s, const std::string & state)
  {
    std::ostringstream out; out << "{\"schema_version\":1,\"label\":" << quoted(label_)
      << ",\"state\":" << quoted(state) << ",\"session_directory\":" << quoted(s.directory.string())
      << ",\"start_ros_ns\":" << s.start_ros << ",\"start_steady_ns\":" << s.start_steady
      << ",\"stop_ros_ns\":" << s.stop_ros << ",\"stop_steady_ns\":" << s.stop_steady
      << ",\"received_messages\":" << s.received << ",\"written_messages\":" << s.written
      << ",\"dropped_messages\":" << s.dropped << ",\"recording_error\":" << quoted(s.error)
      << ",\"ros_domain_id\":" << quoted(std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "0")
      << ",\"topics\":{"; bool first = true;
    for (const auto & t : topics_) {
      if (!first) {out << ',';} first = false;
      const auto count = s.counts.find(t.name);
      out << quoted(t.name) << ":{\"type\":" << quoted(t.type) << ",\"count\":"
          << (count == s.counts.end() ? 0 : count->second) << '}';
    }
    out << "}}\n"; atomic_text(s.directory / "session.json", out.str());
  }
  int export_report(const fs::path & directory)
  {
    std::vector<std::string> args{"python3", exporter_, "export", directory.string()};
    std::vector<char *> argv; for (auto & arg : args) {argv.push_back(arg.data());} argv.push_back(nullptr);
    pid_t pid{}; const int result = posix_spawnp(&pid, "python3", nullptr, nullptr, argv.data(), environ);
    if (result != 0) {return result;}
    int status{}; while (waitpid(pid, &status, 0) < 0) {if (errno != EINTR) {return 127;}}
    return WIFEXITED(status) ? WEXITSTATUS(status) : 128;
  }
  void publish_status()
  {
    if (!rclcpp::ok()) {return;}
    std_msgs::msg::String msg; std::ostringstream out;
    out << "{\"state\":" << quoted(state_) << ",\"directory\":" << quoted(active_ ? active_->directory.string() : last_directory_.string())
      << ",\"received\":" << (active_ ? active_->received.load() : last_received_)
      << ",\"written\":" << (active_ ? active_->written.load() : last_written_)
      << ",\"dropped\":" << (active_ ? active_->dropped.load() : last_dropped_)
      << ",\"error\":" << quoted(active_ && active_->failed ? "recording writer failed; stop to finalize" : last_error_) << '}';
    msg.data = out.str(); status_->publish(msg);
  }
  fs::path root_, last_directory_; std::string exporter_, label_, state_{"idle"}, last_error_;
  bool export_on_stop_; int queue_limit_;
  uint64_t last_received_{0}, last_written_{0}, last_dropped_{0};
  std::vector<std::string> config_paths_;
  std::unique_ptr<RecordingSession> active_;
  std::shared_ptr<rcutils_uint8_array_t> latest_alignment_;
  std::vector<rosbag2_storage::TopicMetadata> topics_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> parameter_clients_;
  std::map<std::string, bool> parameter_requested_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_, stop_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace flapping_uav_gcs
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<flapping_uav_gcs::ExperimentRecorder>();
    rclcpp::spin(node); node->finish(); node.reset();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("experiment_recorder_node"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
