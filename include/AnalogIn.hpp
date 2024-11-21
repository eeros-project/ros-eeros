#pragma once

#include "RosNodeDevice.hpp"
#include <eeros/hal/ScalableInput.hpp>
#include <eeros/control/Signal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <eeros_msgs/msg/analog_signal.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace roseeros {
  
/**
 * Class for an analog input. Reads a topic of type 'eeros_msgs::msg::AnalogSignal' with a
 * single analog signal. The timestamp is taken from the ROS message.
 */
class AnalogIn : public eeros::hal::ScalableInput<double> {
 public:
   /**
    * Creates an analog input as specified in the hardware configuration file under
    * the node '"type": "AnalogIn"'
    *
    * @param id - id given by the parameter "signalId" in the hardware configuration file
    * @param libHandle - hardware wrapper library
    * @param device - name of the ROS node
    * @param subDeviceNumber - number of subdevice, must correspond with subdevice type 'AnalogIn'
    * @param channel - channel number
    * @param scale - the input value is divided by this factor
    * @param offset - before scaling this offset is subtracted from the input value
    * @param rangeMin - the input signal is limited to this lower limit
    * @param rangeMax - the input signal is limited to this upper limit
    * @param unit - a unit can be attached to the signal
    * @param additionalArguments - e.g. "msgType", "topic", "dataField", "queueSize", "callOne"
    */
   AnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
            double scale = 1, double offset = 0, double rangeMin = std::numeric_limits<double>::min(),
            double rangeMax = std::numeric_limits<double>::max(), std::string unit = "",
            std::string additionalArguments = "");
  
  /**
   * Reads the actual value on this input.
   *
   * @return value
   */
  virtual double get() override;

  /**
   * Reads the actual timestamp on this input.
   *
   * @return timestamp
   */
  virtual uint64_t getTimestamp() override;
  
 private:
  void callback (const eeros_msgs::msg::AnalogSignal& msg);
  void callbackJointStatePosition0 (const sensor_msgs::msg::JointState& msg);

  RosNodeDevice* dev;
  rclcpp::Node::SharedPtr rosNodeHandle;
  rclcpp::Subscription<eeros_msgs::msg::AnalogSignal>::SharedPtr subscriber;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber;
  uint32_t subDeviceNumber;
  uint32_t channel;
  double data; 
  timestamp_t timestamp;
  std::string msgType;
  std::string topic;
  std::string dataField;
  int queueSize;
  bool callOne;
};

}

extern "C" {
  eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber,
                                                    uint32_t channel, double scale, double offset, double rangeMin, 
                                                    double rangeMax, std::string unit, std::string additionalArguments);
}
