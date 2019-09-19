#include "../include/AnalogOut.hpp"
#include <iostream>
#include <ros/callback_queue.h>

using namespace halros;

AnalogOut::AnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
                    double scale, double offset, double rangeMin, double rangeMax, std::string unit, std::string additionalArguments) 
    : ScalableOutput<double>(id, libHandle, scale, offset, rangeMin, rangeMax, unit),
      subDeviceNumber(subDeviceNumber),
      channel(channel),
      dev(RosNodeDevice::getDevice(device)),
      rosNodeHandle(dev->getRosNodeHandle()), 
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
    else if	((key=="topic") | (key==" topic")) topic = value;
    else if	((key=="dataField") | (key==" dataField")) dataField = value;
    else if	((key=="queueSize") | (key==" queueSize")) queueSize = std::stoi(value);
    else if	((key=="useSignalInTimestamp") | (key==" useSignalInTimestamp")) {
      if (value=="true") useSignalInTimestamp = true;
      else if (value=="false") 	useSignalInTimestamp = false;
      else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
    } else std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
  }
    
  // 3.) Extend parser by setting callback function
  if (msgType == "std_msgs::Float64") {
    publisher = rosNodeHandle->advertise<std_msgs::Float64>(topic, queueSize);
    setFunction = &stdMsgsFloat64Data;
  } else if ( msgType == "sensor_msgs::LaserScan" ) {
    publisher = rosNodeHandle->advertise<sensor_msgs::LaserScan>(topic, queueSize);
    if (dataField == "angle_min") setFunction = &sensorMsgsLaserScanAngleMin;
    else if (dataField == "angle_max") setFunction = &sensorMsgsLaserScanAngleMax;
    else if (dataField == "angle_increment") setFunction = &sensorMsgsLaserScanAngleIncrement;
    else if (dataField == "time_increment") setFunction = &sensorMsgsLaserScanTimeIncrement;
    else if (dataField == "scan_time") setFunction = &sensorMsgsLaserScanScanTime;
    else if (dataField == "range_min") setFunction = &sensorMsgsLaserScanRangeMin;
    else if (dataField == "range_max") setFunction = &sensorMsgsLaserScanRangeMax;
    else std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
  }
    
  else if ( msgType == "sensor_msgs::JointState" ) {
    publisher = rosNodeHandle->advertise<sensor_msgs::JointState>(topic, queueSize);
    std::cout << "aaaaaaaa"<< dataField << std::endl;
    if (dataField == "effort0") setFunction = &sensorMsgsJointStateEffort0;
    else std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
  } else if (msgType == "") std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
  else std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}

// 4.) Create set function for ROS
// std_msgs::Float64
void AnalogOut::stdMsgsFloat64Data(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  std_msgs::Float64 msg;
  msg.data = static_cast<double>( value );
  publisher.publish(msg);
}

// sensor_msgs::LaserScan
void AnalogOut::sensorMsgsLaserScanAngleMin(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  msg.angle_min = value;
  publisher.publish(msg);
}

void AnalogOut::sensorMsgsLaserScanAngleMax(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  msg.angle_max = value;
  publisher.publish(msg);
}

void AnalogOut::sensorMsgsLaserScanAngleIncrement(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  msg.angle_increment = value;
  publisher.publish(msg);
}

void AnalogOut::sensorMsgsLaserScanTimeIncrement(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  msg.time_increment = value;
  publisher.publish(msg);
}

void AnalogOut::sensorMsgsLaserScanScanTime(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  msg.scan_time = value;
  publisher.publish(msg);
}

void AnalogOut::sensorMsgsLaserScanRangeMin(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  msg.range_min = value;
  publisher.publish(msg);
}

void AnalogOut::sensorMsgsLaserScanRangeMax(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  msg.range_max = value;
  publisher.publish(msg);
}

// sensor_msgs::JointState
void AnalogOut::sensorMsgsJointStateEffort0(const double value, const uint64_t timestamp, const ros::Publisher& publisher) {
  sensor_msgs::JointState msg;
  msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
  std::vector<double> valueTmp (1, value);
  msg.effort = valueTmp;
  publisher.publish(msg);
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
    
  setFunction(data, timestamp, publisher);
}

void AnalogOut::setTimestampSignalIn(uint64_t timestampNs) {
  this->timestampSignalIn = timestampNs;
}

extern "C"{
  eeros::hal::ScalableOutput<double> *createAnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                      uint32_t channel, double scale, double offset, double rangeMin, double rangeMax,
                                                      std::string unit, std::string additionalArguments) {
    return new halros::AnalogOut(id, libHandle, device, subDeviceNumber, channel, scale, offset, rangeMin, rangeMax, unit, additionalArguments);
  }
}
