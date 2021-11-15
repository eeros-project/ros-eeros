#ifndef ROS_EEROS_DIGIN_HPP_
#define ROS_EEROS_DIGIN_HPP_

#include <eeros/hal/Input.hpp>
#include "RosNodeDevice.hpp"
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

namespace roseeros {
  
class DigIn : public eeros::hal::Input<bool> {
 public:
  DigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false,
        std::string additionalArguments = "");
  virtual bool get();
  virtual uint64_t getTimestamp() override;
    
 private:
  // callback functions for ROS
  void sensorMsgsBatteryStatePresent (const sensor_msgs::BatteryState::Type& msg) {
    data = msg.present;
    setTimeStamp(msg.header); 
  }
    
  void setTimeStamp();
  void setTimeStamp(const std_msgs::Header& header);
  void setTimestampFromRosMsgHeader(const std_msgs::Header& header);
  uint64_t timestamp;
  bool inverted;
  RosNodeDevice* dev;
  std::shared_ptr<ros::NodeHandle> rosNodeHandle;
  ros::Subscriber subscriber;
  uint32_t subDeviceNumber;
  uint32_t channel;
  bool data; 
  std::string msgType;
  std::string topic;
  std::string dataField;
  int queueSize;
  bool callOne;
  bool useEerosSystemTime;
};

}

extern "C" {
  eeros::hal::Input<bool> *createDigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted, std::string additionalArguments);
}

#endif /* ROS_EEROS_DIGIN_HPP_ */
