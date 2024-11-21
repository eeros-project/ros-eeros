#include "../include/AnalogOut.hpp"

using namespace roseeros;

AnalogOut::AnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
                    double scale, double offset, double rangeMin, double rangeMax, std::string unit, std::string additionalArguments) 
    : ScalableOutput<double>(id, libHandle, scale, offset, rangeMin, rangeMax, unit),
      dev(RosNodeDevice::getDevice(device)),
      rosNodeHandle(dev->getRosNodeHandle()), 
      subDeviceNumber(subDeviceNumber),
      channel(channel),
      data(NAN) {
    
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
    if ((key=="msgType") | (key==" msgType")) msgType = value;
    else if ((key=="topic") | (key==" topic")) topic = value;
    else if ((key=="dataField") | (key==" dataField")) dataField = value;
    else if ((key=="queueSize") | (key==" queueSize")) queueSize = std::stoi(value);
    else std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
  }

  if (msgType == "eeros_msgs::msg::AnalogSignal") {
    publisher = rosNodeHandle->create_publisher<eeros_msgs::msg::AnalogSignal>(topic, queueSize);
  } else if (msgType == "") std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
  else std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}

// HAL functions
double AnalogOut::get() {
  return data * scale + offset;
}

void AnalogOut::set(double valueSet) {
  data = (valueSet - offset) / scale;
  if(data > maxOut) data = maxOut;
  if(data < minOut) data = minOut;
  eeros_msgs::msg::AnalogSignal msg;
  msg.val = {data};
  msg.timestamp = RosTools::convertToRosTime(timestamp);
  publisher->publish(msg);
}

void AnalogOut::setTimestampSignalIn(uint64_t t) {
  this->timestamp = t;
}

extern "C"{
  eeros::hal::ScalableOutput<double> *createAnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                      uint32_t channel, double scale, double offset, double rangeMin, double rangeMax,
                                                      std::string unit, std::string additionalArguments) {
    return new roseeros::AnalogOut(id, libHandle, device, subDeviceNumber, channel, scale, offset, rangeMin, rangeMax, unit, additionalArguments);
  }
}
