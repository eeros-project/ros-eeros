#pragma once

#include "RosNodeDevice.hpp"
#include <eeros/hal/Output.hpp>
#include <eeros/control/Signal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <eeros_msgs/msg/digital_signal.hpp>

namespace roseeros {
  
/**
 * Class for a digital output. Writes to a topic of type 'eeros_msgs::msg::DigitalSignal' with a
 * single digital signal. The timestamp is taken from the EEROS signal.
 */
class DigOut : public eeros::hal::Output<bool> {
 public:
  /**
   * Creates a digital output as specified in the hardware configuration file under
   * the node '"type": "DigOut"'
   *
   * @param id - id given by the parameter "signalId" in the hardware configuration file
   * @param libHandle - hardware wrapper library
   * @param device - name of the ROS node
   * @param subDeviceNumber - number of subdevice, must correspond with subdevice type 'DigIn'
   * @param channel - channel number
   * @param inverted - if true, logical level is inverted when writing
   * @param additionalArguments - e.g. "msgType", "topic", "dataField", "queueSize"
   */
  DigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
         bool inverted = false, std::string additionalArguments = "");
  
  /**
   * Reads the actual value on this output.
   *
   * @return value
   */
  virtual bool get() override;

  /**
   * Sets the value to be packed into the ROS message.
   *
   * @param value - value
   */
  virtual void set(bool value) override;

  /**
   * Sets the timestamp.
   *
   * @param t - timestamp
   */
  virtual void setTimestampSignalIn(uint64_t t) override;
  
 private:
  RosNodeDevice* dev;
  rclcpp::Node::SharedPtr rosNodeHandle;
  rclcpp::Publisher<eeros_msgs::msg::DigitalSignal>::SharedPtr publisher;
  uint32_t subDeviceNumber;
  uint32_t channel;
  bool data; 
  bool inverted;
  uint64_t timestamp;
};

}

extern "C" {
  eeros::hal::Output<bool> *createDigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, 
                                         uint32_t channel, bool inverted, std::string additionalArguments);
}
