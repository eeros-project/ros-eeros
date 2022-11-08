#ifndef ROS_EEROS_ANALOGIN_HPP_
#define ROS_EEROS_ANALOGIN_HPP_

#include "RosNodeDevice.hpp"

#include <eeros/hal/ScalableInput.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace roseeros {
  
  using std::placeholders::_1;

  class AnalogIn : public eeros::hal::ScalableInput<double> {
  public:
    AnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
             double scale = 1, double offset = 0, double rangeMin = std::numeric_limits<double>::min(),
             double rangeMax = std::numeric_limits<double>::max(), std::string unit = "",
             std::string additionalArguments = "");

    virtual double get();
    virtual uint64_t getTimestamp() override;


  private:
    // 2.) Create new callback functions for ROS
    // /////////////////////////////////////////
    void stdMsgsFloat64Data(const std_msgs::msg::Float64& msg) {
      data = msg.data;
      setTimeStamp();
    };

    void sensorMsgsLaserScanAngleMin(const sensor_msgs::msg::LaserScan& msg) {
      data = msg.angle_min;
      setTimeStamp(msg.header);
    };

    void sensorMsgsLaserScanAngleMax(const sensor_msgs::msg::LaserScan& msg) {
      data = msg.angle_max;
      setTimeStamp(msg.header);
    };

    void sensorMsgsLaserScanAngleIncrement(const sensor_msgs::msg::LaserScan& msg) {
      data = msg.angle_increment;
      setTimeStamp(msg.header);
    };

    void sensorMsgsLaserScanTimeIncrement(const sensor_msgs::msg::LaserScan& msg) {
      data = msg.time_increment;
      setTimeStamp(msg.header);
    };

    void sensorMsgsLaserScanScanTime(const sensor_msgs::msg::LaserScan& msg) {
      data = msg.scan_time;
      setTimeStamp(msg.header);
    };

    void sensorMsgsLaserScanRangeMin(const sensor_msgs::msg::LaserScan& msg) {
      data = msg.range_min;
      setTimeStamp(msg.header);
    };

    void sensorMsgsLaserScanRangeMax(const sensor_msgs::msg::LaserScan& msg) {
      data = msg.range_max;
      setTimeStamp(msg.header);
    };

    void sensorMsgsJointStatePosition0(const sensor_msgs::msg::JointState& msg) {
      data = msg.position[0];
      setTimeStamp(msg.header);
    };

    void setTimeStamp();
    void setTimeStamp(const std_msgs::msg::Header header);
    void setTimestampFromRosMsgHeader(const std_msgs::msg::Header header);
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
};

extern "C" {
  eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                    uint32_t channel, double scale, double offset, double rangeMin, 
                                                    double rangeMax, std::string unit, std::string additionalArguments);
}

#endif /* ROS_EEROS_ANALOGIN_HPP_ */
