#ifndef ROS_EEROS_ANALOGOUT_HPP_
#define ROS_EEROS_ANALOGOUT_HPP_

#include <eeros/hal/ScalableOutput.hpp>
#include <eeros/control/ROS/EerosRosTools.hpp>
#include "RosNodeDevice.hpp"
#include <ros/ros.h>
// 1.) include ROS message type
// ////////////////////////////
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

namespace halros {
	class AnalogOut : public eeros::hal::ScalableOutput<double> {
	public:
		AnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, 
			  double scale = 1, double offset = 0, 
			  double rangeMin = std::numeric_limits<double>::min(), double rangeMax = std::numeric_limits<double>::max(), std::string unit = "",
			  std::string additionalArguments = "");
		
		virtual double get() override;
		virtual void set(double voltage) override;
		virtual void setTimestampSignalIn(uint64_t timestampNs) override;
		
		
	private:
		void (*setFunction) (const double, const uint64_t, const ros::Publisher&);
		
		// 2.) Declare set function for ROS
		// ////////////////////////////////
		static void stdMsgsFloat64Data					(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		
		static void sensorMsgsLaserScanAngleMin			(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		static void sensorMsgsLaserScanAngleMax			(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		static void sensorMsgsLaserScanAngleIncrement	(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		static void sensorMsgsLaserScanTimeIncrement	(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		static void sensorMsgsLaserScanScanTime			(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		static void sensorMsgsLaserScanRangeMin			(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		static void sensorMsgsLaserScanRangeMax			(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		
		static void sensorMsgsJointStateEffort0			(const double value, const uint64_t timestamp, const ros::Publisher& publisher);
		
		
		RosNodeDevice* dev;
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		ros::Subscriber subscriber;
		uint32_t subDeviceNumber;
		uint32_t channel;
		ros::Publisher publisher;
		double data; 
		uint64_t timestampSignalIn;
		std::string msgType;
		std::string topic;
		std::string dataField;
		int queueSize;
		bool callOne;
		bool useSignalInTimestamp;
	};
};

extern "C"{
	eeros::hal::ScalableOutput<double> *createAnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, 
														double scale, double offset, double rangeMin, double rangeMax, std::string unit,
														std::string additionalArguments);
}

#endif /* ROS_EEROS_ANALOGOUT_HPP_ */