#ifndef ROS_NODE_EEROS_DEVICE_HPP_
#define ROS_NODE_EEROS_DEVICE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace roseeros {

static const std::string errorString = "\033[1;31mERROR ros-eeros: \033[0m";

class RosNodeDevice {
 public:
  virtual ~RosNodeDevice() {
    devices.clear();
  }

  static RosNodeDevice* getDevice(std::string rosNode) {
    auto devIt = devices.find(rosNode);
    if (devIt != devices.end()) return devIt->second;
    else return new RosNodeDevice(rosNode);
  }

  rclcpp::Node::SharedPtr getRosNodeHandle() {
    return rosNodeHandle;
  }
 
 private:
  RosNodeDevice(std::string rosNode) {
    rclcpp::init(0, NULL);
    rosNodeHandle = rclcpp::Node::make_shared(rosNode);
    devices[rosNode] = this;
  }

  rclcpp::Node::SharedPtr rosNodeHandle;
  static std::map<std::string, RosNodeDevice*> devices;
};

}

#endif /* ROS_NODE_EEROS_DEVICE_HPP_ */ 
