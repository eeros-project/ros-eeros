#pragma once

#include "RosNodeDevice.hpp"
#include <eeros/hal/ScalableInput.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace roseeros {
  
class AnalogIn : public eeros::hal::ScalableInput<double> {
 public:
  AnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
           double scale = 1, double offset = 0, double rangeMin = std::numeric_limits<double>::min(), 
           double rangeMax = std::numeric_limits<double>::max(), std::string unit = "", std::string additionalArguments = "");
  
  virtual double get() override;
  virtual uint64_t getTimestamp() override;
  
 private:
  void callbackFloat64Data (const std_msgs::msg::Float64& msg) {
    data = msg.data;
    setTimeStamp();
  }
  
  void callbackLaserScanAngleMin (const sensor_msgs::msg::LaserScan& msg) {
    data = msg.angle_min;
    setTimeStamp(msg.header); 
  }

  void callbackLaserScanAngleMax (const sensor_msgs::msg::LaserScan& msg) {
    data = msg.angle_max;
    setTimeStamp(msg.header);

  }

  void callbackLaserScanAngleIncrement (const sensor_msgs::msg::LaserScan& msg) {
    data = msg.angle_increment;
    setTimeStamp(msg.header);
  }
  
  void callbackLaserScanTimeIncrement (const sensor_msgs::msg::LaserScan& msg) {
    data = msg.time_increment;
    setTimeStamp(msg.header); 
  }
  
  void callbackLaserScanScanTime (const sensor_msgs::msg::LaserScan& msg) {
    data = msg.scan_time;
    setTimeStamp(msg.header); 
  }
  
  void callbackLaserScanRangeMin (const sensor_msgs::msg::LaserScan& msg) {
    data = msg.range_min;
    setTimeStamp(msg.header); 
  }
  
  void callbackLaserScanRangeMax (const sensor_msgs::msg::LaserScan& msg) {
    data = msg.range_max;
    setTimeStamp(msg.header); 
  }
  
  void callbackJointStatePosition0 (const sensor_msgs::msg::JointState& msg) {
    data = msg.position[0];
    setTimeStamp(msg.header); 
  }

  void setTimeStamp();
  void setTimeStamp(const std_msgs::msg::Header& header);
  void setTimestampFromRosMsgHeader(const std_msgs::msg::Header& header);
  uint64_t timestamp;

  RosNodeDevice* dev;
  rclcpp::Node::SharedPtr rosNodeHandle;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr float64Subscriber;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScanSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber;
  uint32_t subDeviceNumber;
  uint32_t channel;
  double data; 
  std::string msgType;
  std::string topic;
  std::string dataField;
  int queueSize;
  bool callOne;
  bool useEerosSystemTime;
};

}

extern "C" {
  eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                    uint32_t channel, double scale, double offset, double rangeMin, 
                                                    double rangeMax, std::string unit, std::string additionalArguments);
}
