#include "../include/DigOut.hpp"
#include <eeros/control/ros2/EerosRosTools.hpp>

using namespace roseeros;

DigOut::DigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted, std::string additionalArguments)
    : eeros::hal::Output<bool>(id, libHandle),
      dev(RosNodeDevice::getDevice(device)),
      subDeviceNumber(subDeviceNumber),
      channel(channel),
      rosNodeHandle(dev->getRosNodeHandle()),
      data(false),
      queueSize(1000),
      callOne(true),
      inverted(inverted) {
  // parsing additionalArguments:
  auto s = additionalArguments;
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
    else if ((key=="useSignalInTimestamp") | (key==" useSignalInTimestamp")) {
      if (value=="true") useSignalInTimestamp = true;
      else if	(value=="false") useSignalInTimestamp = false;
      else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
    } else std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
  }

  // selecting callback function for ros
  if (msgType == "sensor_msgs::msg::BatteryState") {
    publisher = rosNodeHandle->create_publisher<sensor_msgs::msg::BatteryState>(topic, queueSize);
    if (dataField == "present") setFunction = &callback;
    else
      std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
  } else if ( msgType == "" )
    std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
  else
    std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}

void DigOut::callback(const DigOut& self, const bool value, const uint64_t timestamp){
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  msg.present = static_cast<uint8_t>( value );
  self.publisher-> publish(msg);
}

bool DigOut::get() {
  if(inverted) return !data;
  return data;
}

void DigOut::set(bool value) {
  if(inverted) value = !value;
  data = value;
  uint64_t timestamp;
  if (useSignalInTimestamp) timestamp = timestampSignalIn;
  else timestamp = eeros::System::getTimeNs();
  setFunction(*this, value, timestamp);
}

void DigOut::setTimestampSignalIn(uint64_t timestampNs) {
  this->timestampSignalIn = timestampNs;
}

extern "C" eeros::hal::Output<bool> *createDigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                  uint32_t channel, bool inverted, std::string additionalArguments) {
  return new roseeros::DigOut(id, libHandle, device, subDeviceNumber, channel, inverted, additionalArguments);
}
