#ifndef ROS_NODE_EEROS_DEVICE_HPP_
#define ROS_NODE_EEROS_DEVICE_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <ros/callback_queue.h>
// #include <thread>
// #include "SimChannel.hpp"
// #include "Reflect.hpp"

namespace halros {
	
	static const std::string errorString = "\033[1;31mERROR ros-eeros: \033[0m";
	
	class RosNodeDevice {
	public:
		virtual ~RosNodeDevice();

 		static RosNodeDevice* getDevice(std::string rosNode);
		std::shared_ptr<ros::NodeHandle> getRosNodeHandle();
// 		subscribeTopic(
		
	private:
		RosNodeDevice(std::string rosNode);
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		static std::map<std::string, halros::RosNodeDevice *> devices;
// 		static halros::RosNodeDevice* instance;
	};
};

#endif /* ROS_NODE_EEROS_DEVICE_HPP_ */ 