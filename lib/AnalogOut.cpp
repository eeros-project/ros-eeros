#include "../include/AnalogOut.hpp"

using namespace roseeros;

AnalogOut::AnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
                    double scale, double offset, double rangeMin, double rangeMax, std::string unit, std::string additionalArguments) 
    : ScalableOutput<double>(id, libHandle, scale, offset, rangeMin, rangeMax, unit),
      dev(RosNodeDevice::getDevice(device)),
      rosNodeHandle(dev->getRosNodeHandle()), 
      subDeviceNumber(subDeviceNumber),
      channel(channel),
      data(NAN),
      queueSize(1000),
      callOne(true),
      useSignalInTimestamp(false) {
    
  // parsing additionalArguments:
  auto s = additionalArguments;
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
    else if ((key=="useSignalInTimestamp") | (key==" useSignalInTimestamp")) {
      if (value=="true") useSignalInTimestamp = true;
      else if (value=="false") useSignalInTimestamp = false;
      else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
    } else std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
  }
    
  // set callback function
  if (msgType == "std_msgs::msg::Float64") {
    float64Publisher = rosNodeHandle->create_publisher<std_msgs::msg::Float64>(topic, queueSize);
    setFunction = &callbackFloat64Data;
  } else if (msgType == "sensor_msgs::LaserScan") {
    laserScanPublisher = rosNodeHandle->create_publisher<sensor_msgs::msg::LaserScan>(topic, queueSize);
    if (dataField == "angle_min") setFunction = &callbackLaserScanAngleMin;
    else if (dataField == "angle_max") setFunction = &callbackLaserScanAngleMax;
    else if (dataField == "angle_increment") setFunction = &callbackLaserScanAngleIncrement;
    else if (dataField == "time_increment") setFunction = &callbackLaserScanTimeIncrement;
    else if (dataField == "scan_time") setFunction = &callbackLaserScanScanTime;
    else if (dataField == "range_min") setFunction = &callbackLaserScanRangeMin;
    else if (dataField == "range_max") setFunction = &callbackLaserScanRangeMax;
    else std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
  } else if (msgType == "sensor_msgs::msg::JointState") {
    jointStatePublisher = rosNodeHandle->create_publisher<sensor_msgs::msg::JointState>(topic, queueSize);
    if (dataField == "effort0") setFunction = &callbackJointStateEffort0;
    else std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
  } else if (msgType == "") std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
  else std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}

void AnalogOut::callbackFloat64Data(const AnalogOut& self, const double value, const uint64_t timestamp) {
  std_msgs::msg::Float64 msg;
  msg.data = static_cast<double>( value );
  self.float64Publisher->publish(msg);
}

// sensor_msgs::LaserScan
void AnalogOut::callbackLaserScanAngleMin(const AnalogOut& self, const double value, const uint64_t timestamp) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = eeros::control::RosTools::convertToRosTime(timestamp);
  msg.angle_min = value;
  self.laserScanPublisher->publish(msg);
}

void AnalogOut::callbackLaserScanAngleMax(const AnalogOut& self, const double value, const uint64_t timestamp) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = eeros::control::RosTools::convertToRosTime(timestamp);
  msg.angle_max = value;
  self.laserScanPublisher->publish(msg);
}

void AnalogOut::callbackLaserScanAngleIncrement(const AnalogOut& self, const double value, const uint64_t timestamp) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = eeros::control::RosTools::convertToRosTime(timestamp);
  msg.angle_increment = value;
  self.laserScanPublisher->publish(msg);
}

void AnalogOut::callbackLaserScanTimeIncrement(const AnalogOut& self, const double value, const uint64_t timestamp) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = eeros::control::RosTools::convertToRosTime(timestamp);
  msg.time_increment = value;
  self.laserScanPublisher->publish(msg);
}

void AnalogOut::callbackLaserScanScanTime(const AnalogOut& self, const double value, const uint64_t timestamp) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = eeros::control::RosTools::convertToRosTime(timestamp);
  msg.scan_time = value;
  self.laserScanPublisher->publish(msg);
}

void AnalogOut::callbackLaserScanRangeMin(const AnalogOut& self, const double value, const uint64_t timestamp) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = eeros::control::RosTools::convertToRosTime(timestamp);
  msg.range_min = value;
  self.laserScanPublisher->publish(msg);
}

void AnalogOut::callbackLaserScanRangeMax(const AnalogOut& self, const double value, const uint64_t timestamp) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = eeros::control::RosTools::convertToRosTime(timestamp);
  msg.range_max = value;
  self.laserScanPublisher->publish(msg);
}

void AnalogOut::callbackJointStateEffort0(const AnalogOut& self, const double value, const uint64_t timestamp) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = eeros::control::RosTools::convertToRosTime(timestamp);
  std::vector<double> valueTmp (1, value);
  msg.effort = valueTmp;
  self.jointStatePublisher->publish(msg);
}


// HAL functions
double AnalogOut::get() {
  return data * scale + offset;
}

void AnalogOut::set(double valueSet) {
  data = (valueSet - offset) / scale;
  if(data > maxOut) data = maxOut;
  if(data < minOut) data = minOut;
  
  uint64_t timestamp;
  if (useSignalInTimestamp) timestamp = timestampSignalIn;
  else timestamp = eeros::System::getTimeNs();
    
  setFunction(*this, data, timestamp);
}

void AnalogOut::setTimestampSignalIn(uint64_t timestampNs) {
  this->timestampSignalIn = timestampNs;
}

extern "C"{
  eeros::hal::ScalableOutput<double> *createAnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                      uint32_t channel, double scale, double offset, double rangeMin, double rangeMax,
                                                      std::string unit, std::string additionalArguments) {
    return new roseeros::AnalogOut(id, libHandle, device, subDeviceNumber, channel, scale, offset, rangeMin, rangeMax, unit, additionalArguments);
  }
}
