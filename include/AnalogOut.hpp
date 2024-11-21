#pragma once

#include "RosNodeDevice.hpp"
#include <rclcpp/rclcpp.hpp>
#include <eeros/hal/ScalableOutput.hpp>
#include <eeros_msgs/msg/analog_signal.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace roseeros {

/**
 * Class for an analog output. Writes to a topic of type 'eeros_msgs::msg::AnalogSignal' with a
 * single analog signal. The timestamp is taken from the EEROS signal.
 */
class AnalogOut : public eeros::hal::ScalableOutput<double> {
 public:
  /**
   * Creates an analog output as specified in the hardware configuration file under
   * the node '"type": "AnalogOut"'
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
   * @param additionalArguments - e.g. "msgType", "topic", "dataField", "queueSize"
   */
  AnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
            double scale = 1, double offset = 0, double rangeMin = std::numeric_limits<double>::min(),
            double rangeMax = std::numeric_limits<double>::max(), std::string unit = "",
            std::string additionalArguments = "");
  /**
   * Reads the actual value on this output.
   *
   * @return value
   */
  virtual double get() override;

  /**
   * Sets the value to be packed into the ROS message.
   *
   * @param value - value
   */
  virtual void set(double voltage) override;

  /**
   * Sets the timestamp.
   *
   * @param t - timestamp
   */
  virtual void setTimestampSignalIn(uint64_t t) override;
  
 private:
  RosNodeDevice* dev;
  rclcpp::Node::SharedPtr rosNodeHandle;
  rclcpp::Publisher<eeros_msgs::msg::AnalogSignal>::SharedPtr publisher;
  uint32_t subDeviceNumber;
  uint32_t channel;
  double data; 
  uint64_t timestamp;
};

  
};

extern "C"{
  eeros::hal::ScalableOutput<double> *createAnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, 
                                                      double scale, double offset, double rangeMin, double rangeMax, std::string unit,
                                                      std::string additionalArguments);
}
