#include "../include/AnalogIn.hpp"

using namespace roseeros;
using std::placeholders::_1;

AnalogIn::AnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
                   double scale, double offset, double rangeMin, double rangeMax, std::string unit,
                   std::string additionalArguments) 
    : ScalableInput<double>(id, libHandle, scale, offset, rangeMin, rangeMax, unit),
      dev(RosNodeDevice::getDevice(device)), rosNodeHandle(dev->getRosNodeHandle()),
      subDeviceNumber(subDeviceNumber), channel(channel), data(0), queueSize(1000), callOne(true), useEerosSystemTime(false) {
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
    else if	((key=="queueSize") | (key==" queueSize"))
      queueSize = std::stoi(value);
    else if ((key=="useEerosSystemTime") | (key==" useEerosSystemTime")) {
      if (value=="true") useEerosSystemTime = true;
      else if (value=="false") useEerosSystemTime = false;
      else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
    } else if ((key=="callOne") | (key==" callOne")) {
      if (value=="true") callOne = true;
      else if (value=="false") callOne = false;
      else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
    } else
      std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
  }
  
  if (msgType == "std_msgs::msg::Float64") {
    float64Subscriber = rosNodeHandle->create_subscription<std_msgs::msg::Float64>(topic, queueSize, std::bind(&AnalogIn::callbackFloat64Data, this, _1));
  } else if ( msgType == "sensor_msgs::msg::LaserScan" ) {
    if ( dataField == "angle_min" )
      laserScanSubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::LaserScan>(topic, queueSize, std::bind(&AnalogIn::callbackLaserScanAngleMin, this, _1));
    else if ( dataField == "angle_max" )
      laserScanSubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::LaserScan>(topic, queueSize, std::bind(&AnalogIn::callbackLaserScanAngleMax, this, _1));
    else if ( dataField == "angle_increment" )
      laserScanSubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::LaserScan>(topic, queueSize, std::bind(&AnalogIn::callbackLaserScanAngleIncrement, this, _1));
    else if ( dataField == "time_increment" )
      laserScanSubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::LaserScan>(topic, queueSize, std::bind(&AnalogIn::callbackLaserScanTimeIncrement, this, _1));
    else if ( dataField == "scan_time" )
      laserScanSubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::LaserScan>(topic, queueSize, std::bind(&AnalogIn::callbackLaserScanScanTime, this, _1));
    else if ( dataField == "range_min" )
      laserScanSubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::LaserScan>(topic, queueSize, std::bind(&AnalogIn::callbackLaserScanRangeMin, this, _1));
    else if ( dataField == "range_max" )
      laserScanSubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::LaserScan>(topic, queueSize, std::bind(&AnalogIn::callbackLaserScanRangeMax, this, _1));
    else
      std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
  } else if ( msgType == "sensor_msgs::msg::JointState" ) {
    if ( dataField == "position0" )
      jointStateSubscriber = rosNodeHandle->create_subscription<sensor_msgs::msg::JointState>(topic, queueSize, std::bind(&AnalogIn::callbackJointStatePosition0, this, _1));
    else
      std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
  } else if ( msgType == "" )
    std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
  else 
    std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}

double AnalogIn::get() {
  if (callOne)
    rclcpp::spin_some(rosNodeHandle); // calls callback fct. only for the oldest message
  else
    rclcpp::spin(rosNodeHandle); // calls callback fct. for all available messages.
                              // Only newest message is processed. Older ones are discarded.	
  double inVal = (data - offset) / scale;
  if (inVal > maxIn) inVal = maxIn;
  if (inVal < minIn) inVal = minIn;
  return inVal;
}

uint64_t AnalogIn::getTimestamp() {
  return timestamp;
}

void AnalogIn::setTimeStamp() {
  timestamp = eeros::System::getTimeNs();
}

void AnalogIn::setTimeStamp(const std_msgs::msg::Header& header) {
  if (useEerosSystemTime) setTimeStamp();
  else setTimestampFromRosMsgHeader(header);
  
}

void AnalogIn::setTimestampFromRosMsgHeader(const std_msgs::msg::Header& header) {
  timestamp = eeros::control::RosTools::toNanoSec(header.stamp);
}


extern "C"{
  eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, 
              void* libHandle, 
              std::string device, 
              uint32_t subDeviceNumber, 
              uint32_t channel, 
              double scale, 
              double offset, 
              double rangeMin, 
              double rangeMax, 
              std::string unit,
              std::string additionalArguments){
    return new roseeros::AnalogIn(id, libHandle, device, subDeviceNumber, channel, scale, offset, rangeMin, rangeMax, unit, additionalArguments);
  }
}

