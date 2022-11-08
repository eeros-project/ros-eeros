#ifndef ROS_EEROS_ANALOGOUT_HPP_
#define ROS_EEROS_ANALOGOUT_HPP_

#include "RosNodeDevice.hpp"

#include <eeros/hal/ScalableOutput.hpp>
#include <eeros/control/ros/EerosRosTools.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace roseeros {
  
class AnalogOut : public eeros::hal::ScalableOutput<double> {
 public:
  AnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0, 
            double rangeMin = std::numeric_limits<double>::min(), double rangeMax = std::numeric_limits<double>::max(), std::string unit = "",
            std::string additionalArguments = "");
  virtual double get() override;
  virtual void set(double voltage) override;
  virtual void setTimestampSignalIn(uint64_t timestampNs) override;
  
 private:
  void (*setFunction) (const AnalogOut& self, const double, const uint64_t);

  // 2.) Declare set function for ROS
  static void stdMsgsFloat64Data(const AnalogOut& self, const double value, const uint64_t timestamp);

  static void sensorMsgsLaserScanAngleMin(const AnalogOut& self, const double value, const uint64_t timestamp);
  static void sensorMsgsLaserScanAngleMax(const AnalogOut& self, const double value, const uint64_t timestamp);
  static void sensorMsgsLaserScanAngleIncrement(const AnalogOut& self, const double value, const uint64_t timestamp);
  static void sensorMsgsLaserScanTimeIncrement(const AnalogOut& self, const double value, const uint64_t timestamp);
  static void sensorMsgsLaserScanScanTime(const AnalogOut& self, const double value, const uint64_t timestamp);
  static void sensorMsgsLaserScanRangeMin(const AnalogOut& self, const double value, const uint64_t timestamp);
  static void sensorMsgsLaserScanRangeMax(const AnalogOut& self, const double value, const uint64_t timestamp);

  static void sensorMsgsJointStateEffort0(const AnalogOut& self, const double value, const uint64_t timestamp);
  
  RosNodeDevice* dev;
  rclcpp::Node::SharedPtr rosNodeHandle;
  uint32_t subDeviceNumber;
  uint32_t channel;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr float64Publisher;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserScanPublisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher;
  double data;
  uint64_t timestampSignalIn;
  std::string msgType;
  std::string topic;
  std::string dataField;
  int queueSize;
  bool callOne;
  bool useSignalInTimestamp;
};

  
};

extern "C"{
  eeros::hal::ScalableOutput<double> *createAnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, 
                                                      double scale, double offset, double rangeMin, double rangeMax, std::string unit,
                                                      std::string additionalArguments);
}

#endif /* ROS_EEROS_ANALOGOUT_HPP_ */
