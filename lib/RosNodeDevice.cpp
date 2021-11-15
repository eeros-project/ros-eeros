#include "../include/RosNodeDevice.hpp"

using namespace roseeros;

std::map<std::string, RosNodeDevice *> RosNodeDevice::devices;

RosNodeDevice::RosNodeDevice(std::string rosNode) {
  char* dummyArgs[] = {NULL};
  int dummyArgc = sizeof(dummyArgs) / sizeof(dummyArgs[0]) - 1;
  ros::init(dummyArgc, dummyArgs, rosNode);
  ros::NodeHandle n;
  rosNodeHandle = std::make_shared<ros::NodeHandle>(n);
  devices[rosNode] = this;
}

RosNodeDevice::~RosNodeDevice() {
  devices.clear();
}

std::shared_ptr<ros::NodeHandle> RosNodeDevice::getRosNodeHandle() {
  return rosNodeHandle;
}

RosNodeDevice* RosNodeDevice::getDevice(std::string rosNode) {
  auto devIt = devices.find(rosNode);
  if (devIt != devices.end()) return devIt->second;
  else return new RosNodeDevice(rosNode);
}
