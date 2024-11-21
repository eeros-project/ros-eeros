#include "../include/DigOut.hpp"

using namespace roseeros;

DigOut::DigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted, std::string additionalArguments)
    : eeros::hal::Output<bool>(id, libHandle),
      dev(RosNodeDevice::getDevice(device)),
      rosNodeHandle(dev->getRosNodeHandle()),
      subDeviceNumber(subDeviceNumber),
      channel(channel),
      data(false),
      inverted(inverted) {
  // parsing additionalArguments:
  auto s = additionalArguments;
  std::string msgType;
  std::string topic;
  std::string dataField;
  int queueSize = 1000;
  bool stop = false;
  while(!stop) {
    if(s.find(";") == std::string::npos) stop=true;
    std::string statement = s.substr(0, s.find(";"));
    std::string key = statement.substr(0, statement.find("="));
    std::string value = statement.substr(statement.find("=")+1);
    s = s.substr(s.find(";")+1);
    if ((key=="msgType") | (key==" msgType"))
      msgType = value;
    else if ((key=="topic") | (key==" topic"))
      topic = value;
    else if ((key=="dataField") | (key==" dataField"))
      dataField = value;
    else if ((key=="queueSize") | (key==" queueSize"))
      queueSize = std::stoi(value);
    else std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
  }

  // selecting callback function for ros
  if (msgType == "eeros_msgs::msg::DigitalSignal") {
    publisher = rosNodeHandle->create_publisher<eeros_msgs::msg::DigitalSignal>(topic, queueSize);
  } else if ( msgType == "" )
    std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
  else
    std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}

bool DigOut::get() {
  return data;
}

void DigOut::set(bool value) {
  if(inverted) value = !value;
  data = value;
  eeros_msgs::msg::DigitalSignal msg;
  msg.timestamp = eeros::control::RosTools::convertToRosTime(timestamp);
  msg.val = {value};
  publisher-> publish(msg);
}

void DigOut::setTimestampSignalIn(uint64_t t) {
  this->timestamp = t;
}

extern "C" eeros::hal::Output<bool> *createDigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                  uint32_t channel, bool inverted, std::string additionalArguments) {
  return new roseeros::DigOut(id, libHandle, device, subDeviceNumber, channel, inverted, additionalArguments);
}
