#pragma once

#include "RosNodeDevice.hpp"
#include <eeros/hal/Input.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace roseeros {
  
/**
 * Class for a digital input
 */
class DigIn : public eeros::hal::Input<bool> {
 public:
  /**
   * Creates a digital input as specified in the hardware configuration file under
   * the node '"type": "DigIn"'
   *
   * @param id - id given by the parameter "signalId" in the hardware configuration file
   * @param libHandle - hardware wrapper library
   * @param device - name of the ROS node
   * @param subDeviceNumber - number of
   * @param channel - id given by the parameter "signalId" in the hardware configuration file
   * @param inverted - id given by the parameter "signalId" in the hardware configuration file
   */
  DigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false,
        std::string additionalArguments = "");
  virtual bool get() override;
  virtual uint64_t getTimestamp() override;
    
 private:
  void callback(const sensor_msgs::msg::BatteryState& msg); // callback functions for ROS
  void setTimeStamp();
  void setTimeStamp(const std_msgs::msg::Header& header);
  void setTimestampFromRosMsgHeader(const std_msgs::msg::Header& header);

  uint64_t timestamp;
  RosNodeDevice* dev;
  rclcpp::Node::SharedPtr rosNodeHandle;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscriber;
  uint32_t subDeviceNumber;
  uint32_t channel;
  bool data;
  bool inverted;
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
