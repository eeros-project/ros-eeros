#pragma once

#include "RosNodeDevice.hpp"
#include <eeros/hal/Output.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace roseeros {
  
/**
 *
 */
class DigOut : public eeros::hal::Output<bool> {
 public:
  /**
   *
   */
  DigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
         bool inverted = false, std::string additionalArguments = "");
  
  virtual bool get() override;
  virtual void set(bool value) override;
  virtual void setTimestampSignalIn(uint64_t timestampNs) override;
  
 private:
  void (*setFunction) (const DigOut& self, const bool value, const uint64_t timestamp);
  static void callback(const DigOut& self, const bool value, const uint64_t timestamp);
  
  RosNodeDevice* dev;
  rclcpp::Node::SharedPtr rosNodeHandle;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher;
  uint32_t subDeviceNumber;
  uint32_t channel;
  bool data; 
  bool inverted;
  uint64_t timestampSignalIn;
  std::string msgType;
  std::string topic;
  std::string dataField;
  int queueSize;
  bool callOne;
  bool useSignalInTimestamp;
};

}

extern "C" {
  eeros::hal::Output<bool> *createDigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, 
                                         uint32_t channel, bool inverted, std::string additionalArguments);
}
