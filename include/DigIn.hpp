#pragma once

#include "RosNodeDevice.hpp"
#include <eeros/hal/Input.hpp>
#include <eeros/control/Signal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <eeros_msgs/msg/digital_signal.hpp>

namespace roseeros {
  
/**
 * Class for a digital input. Reads a topic of type 'eeros_msgs::msg::DigitalSignal' with a
 * single digital signal. The timestamp is taken from the ROS message.
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
   * @param subDeviceNumber - number of subdevice, must correspond with subdevice type 'DigIn'
   * @param channel - channel number
   * @param inverted - if true, logical level is inverted when reading
   * @param additionalArguments - e.g. "msgType", "topic", "dataField", "queueSize", "callOne"
   */
  DigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
        bool inverted = false, std::string additionalArguments = "");

  /**
   * Reads the actual value on this input.
   *
   * @return value
   */
  virtual bool get() override;

  /**
   * Reads the actual timestamp on this input.
   *
   * @return timestamp
   */
  virtual uint64_t getTimestamp() override;
    
 private:
  void callback(const eeros_msgs::msg::DigitalSignal& msg); // callback functions for ROS

  RosNodeDevice* dev;
  rclcpp::Node::SharedPtr rosNodeHandle;
  rclcpp::Subscription<eeros_msgs::msg::DigitalSignal>::SharedPtr subscriber;
  uint32_t subDeviceNumber;
  uint32_t channel;
  bool data;
  bool inverted;
  timestamp_t timestamp;
};

}

extern "C" {
  eeros::hal::Input<bool> *createDigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted, std::string additionalArguments);
}
