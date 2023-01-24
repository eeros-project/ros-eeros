#include "../include/RosNodeDevice.hpp"

using namespace roseeros;

std::map<std::string, RosNodeDevice *> RosNodeDevice::devices;

RosNodeDevice::RosNodeDevice(std::string rosNode) {
  rclcpp::init(0, NULL);
  rosNodeHandle = rclcpp::Node::make_shared(rosNode);
  devices[rosNode] = this;
}

RosNodeDevice::~RosNodeDevice() {
  devices.clear();
}

rclcpp::Node::SharedPtr RosNodeDevice::getRosNodeHandle() {
  return rosNodeHandle;
}

RosNodeDevice* RosNodeDevice::getDevice(std::string rosNode) {
  auto devIt = devices.find(rosNode);
  if (devIt != devices.end()) return devIt->second;
  else return new RosNodeDevice(rosNode);
}
