// Lifecycle host for the pinned official LinkTrack implementation.
// Protocol/serial parsing remains in unchanged upstream sources. Foxy's RMW
// publisher handles must be released while their owning node is still alive.
#include <atomic>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "linktrack_node/linktrackinit.h"
#include "nlink_utils/init_serial.h"
#include "nprotocol_extracter.h"

extern asio::io_context g_ioc;

class SerialWorker
{
public:
  SerialWorker()
  : worker_([this]() {
        try {g_ioc.run();} catch (...) {error_ = std::current_exception(); failed_.store(true);}
      }) {}

  ~SerialWorker() {stop();}
  void stop()
  {
    g_ioc.stop();
    if (worker_.joinable()) {worker_.join();}
  }
  bool failed() const {return failed_.load();}
  void rethrow_if_failed() const {if (error_) {std::rethrow_exception(error_);}}

private:
  std::exception_ptr error_;
  std::atomic<bool> failed_{false};
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;
  int result = 0;
  try {
    node = std::make_shared<rclcpp::Node>("linktrack_parser");
    NProtocolExtracter extraction;
    auto serial = initSerial(
      node, [&extraction](const std::string & bytes) {
        extraction.AddNewData(bytes);
      });
    linktrack::Init upstream(node, &extraction, serial.get());
    SerialWorker worker;
    rclcpp::WallRate rate(1000);
    while (rclcpp::ok() && !worker.failed()) {
      rclcpp::spin_some(node);
      rate.sleep();
    }
    worker.stop();
    worker.rethrow_if_failed();
  } catch (const std::exception & error) {
    std::cerr << "linktrack_host_node: " << error.what() << '\n';
    result = 1;
  }
  // The upstream global publisher map otherwise outlives the local node and
  // crashes in __rmw_destroy_publisher during static destruction on Foxy.
  linktrack::publishers_.clear();
  node.reset();
  rclcpp::shutdown();
  return result;
}
