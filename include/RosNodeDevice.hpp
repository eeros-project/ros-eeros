#ifndef ROS_NODE_EEROS_DEVICE_HPP_
#define ROS_NODE_EEROS_DEVICE_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>

namespace roseeros {

static const std::string errorString = "\033[1;31mERROR ros-eeros: \033[0m";

class RosNodeDevice {
  public:
    virtual ~RosNodeDevice();

    static RosNodeDevice* getDevice(std::string rosNode);
    rclcpp::Node::SharedPtr getRosNodeHandle();

  private:
    RosNodeDevice(std::string rosNode);
    rclcpp::Node::SharedPtr rosNodeHandle;
    static std::map<std::string, RosNodeDevice*> devices;
};

}

#endif /* ROS_NODE_EEROS_DEVICE_HPP_ */ 
