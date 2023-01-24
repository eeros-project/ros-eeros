#ifndef ROS_EEROS_DIGIN_HPP_
#define ROS_EEROS_DIGIN_HPP_

#include "RosNodeDevice.hpp"

#include <string>
#include <memory>

#include <eeros/hal/Input.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace roseeros {

  using std::placeholders::_1;

  class DigIn : public eeros::hal::Input<bool> {
  public:
    DigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
          uint32_t channel, bool inverted = false, std::string additionalArguments = "");

    virtual bool get();
    virtual uint64_t getTimestamp() override;

  private:
    // callback functions for ROS
    void sensorMsgsBatteryStatePresent(const sensor_msgs::msg::BatteryState msg) {
      data = msg.present;
      setTimeStamp(msg.header);
    };

    void setTimeStamp();
    void setTimeStamp(const std_msgs::msg::Header header);
    void setTimestampFromRosMsgHeader(const std_msgs::msg::Header header);
    uint64_t timestamp;

    bool inverted;
    RosNodeDevice* dev;
    rclcpp::Node::SharedPtr rosNodeHandle;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batterySubscriber;
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
};

extern "C"{
  eeros::hal::Input<bool> *createDigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                       uint32_t channel, bool inverted, std::string additionalArguments);
}

#endif /* ROS_EEROS_DIGIN_HPP_ */
