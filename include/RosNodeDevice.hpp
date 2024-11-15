#pragma once

#include <eeros/control/ros2/RosTools.hpp>

using namespace eeros::control;

namespace roseeros {

static const std::string errorString = "\033[1;31mERROR ros-eeros: \033[0m";

class RosNodeDevice {
 public:
  virtual ~RosNodeDevice() {
    devices.clear();
  }

  static RosNodeDevice* getDevice(std::string name) {
//     std::cout << "trying to get ROS node device with name " << name << std::endl;
    auto devIt = devices.find(name);
    if (devIt != devices.end()) return devIt->second;
    else return new RosNodeDevice(name);
  }

  rclcpp::Node::SharedPtr getRosNodeHandle() {
    return node;
  }
 
 private:
  RosNodeDevice(std::string name) {
//     std::cout << "make a new ROS node device with name " << name << std::endl;
    RosTools::initRos(0, nullptr);
    node = RosTools::initNode(name);
    devices[name] = this;
  }

  rclcpp::Node::SharedPtr node;
  static std::map<std::string, RosNodeDevice*> devices;
};

}
