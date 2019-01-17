#ifndef ROS_EEROS_DIGOUT_HPP_
#define ROS_EEROS_DIGOUT_HPP_

#include <string>
#include <eeros/hal/Output.hpp>
#include <eeros/control/ros/EerosRosTools.hpp>
#include "RosNodeDevice.hpp"
#include <ros/ros.h>
// ROS message types
#include <sensor_msgs/BatteryState.h>

namespace halros {
	class DigOut : public eeros::hal::Output<bool> {
	public:
		DigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
			   bool inverted = false, std::string additionalArguments = "");
		
		virtual bool get() override;
		virtual void set(bool value) override;
		virtual void setTimestampSignalIn(uint64_t timestampNs) override;
		
		
	private:
		void (*setFunction) (const bool, const uint64_t timestamp, const ros::Publisher&);
		
		// set functions for ROS
		// /////////////////////
		static void sensorMsgsBatteryStatePresent		(const bool value, const uint64_t timestamp, const ros::Publisher& publisher);
		
		
		RosNodeDevice* dev;
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		ros::Subscriber subscriber;
		uint32_t subDeviceNumber;
		uint32_t channel;
		ros::Publisher publisher;
		bool data; 
		uint64_t timestampSignalIn;
		std::string msgType;
		std::string topic;
		std::string dataField;
		int queueSize;
		bool callOne;
		bool useSignalInTimestamp;
		
		bool inverted;
	};
};

extern "C"{
	eeros::hal::Output<bool> *createDigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted, std::string additionalArguments);
}

#endif /* ROS_EEROS_DIGOUT_HPP_ */
