#include "../include/DigIn.hpp"
#include <eeros/control/ros/EerosRosTools.hpp>

using namespace roseeros;

DigIn::DigIn(std::string id,
             void* libHandle,
             std::string device,
             uint32_t subDeviceNumber,
             uint32_t channel,
             bool inverted,
             std::string additionalArguments
             ) :
             eeros::hal::Input<bool>(id, libHandle),
             inverted(inverted),
             dev(RosNodeDevice::getDevice(device)),
             rosNodeHandle(dev->getRosNodeHandle()),
             subDeviceNumber(subDeviceNumber),
             channel(channel),
             data(false),
             queueSize(1000),
             callOne(true),
             useEerosSystemTime(false) {

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
    else if ((key=="useEerosSystemTime") | (key==" useEerosSystemTime")) {
      if (value=="true") useEerosSystemTime = true;
      else if (value=="false") useEerosSystemTime = false;
      else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
    }
    else if ((key=="callOne") | (key==" callOne")) {
      if (value=="true") callOne = true;
      else if (value=="false") callOne = false;
      else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
    }
    else
      std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
  }

  // selecting callback function for ros
  if( msgType == "sensor_msgs::BatteryState" ) {
    batterySubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::BatteryState>(topic, queueSize,
                                                                                           std::bind(&DigIn::sensorMsgsBatteryStatePresent, this, _1));
  }
  else if ( msgType == "" )
    std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
  else
    std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}

bool DigIn::get() {
  if(inverted) return !data;
  return data;
}

uint64_t DigIn::getTimestamp() {
  return timestamp;
}

void DigIn::setTimeStamp() {
  timestamp = eeros::System::getTimeNs();
}

void DigIn::setTimeStamp(const std_msgs::msg::Header header) {
  if (useEerosSystemTime) setTimeStamp();
  else setTimestampFromRosMsgHeader(header);
}

void DigIn::setTimestampFromRosMsgHeader(const std_msgs::msg::Header header) {
  timestamp = eeros::control::rosTools::toNanoSec(header.stamp);
}

extern "C" eeros::hal::Input<bool> *createDigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                uint32_t channel, bool inverted, std::string additionalArguments) {
  return new roseeros::DigIn(id, libHandle, device, subDeviceNumber, channel, inverted, additionalArguments);
}
