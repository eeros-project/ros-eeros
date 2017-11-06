#ifndef ROS_EEROS_ANALOGIN_HPP_
#define ROS_EEROS_ANALOGIN_HPP_

#include <eeros/hal/ScalableInput.hpp>
#include "RosNodeDevice.hpp"
#include <ros/ros.h>
// 1.) Include ROS message type
// ////////////////////////////
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

namespace halros {
	class AnalogIn : public eeros::hal::ScalableInput<double> {
	public:
		AnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
				 double scale = 1, double offset = 0,
		   double rangeMin = std::numeric_limits<double>::min(), double rangeMax = std::numeric_limits<double>::max(), std::string unit = "",
				 std::string additionalArguments = "");
		
		virtual double get();
		virtual uint64_t getTimestamp() override;
		
		
	private:
		// 2.) Create new callback functions for ROS
		// /////////////////////////////////////////
		void stdMsgsFloat64Data					(const std_msgs::Float64::Type& msg) 		{data = msg.data;
																							setTimeStamp(); } ;
		
		void sensorMsgsLaserScanAngleMin		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_min;
																							 setTimeStamp(msg.header); } ;
		void sensorMsgsLaserScanAngleMax		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_max;
																							 setTimeStamp(msg.header); } ;
		void sensorMsgsLaserScanAngleIncrement	(const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_increment;
																							 setTimeStamp(msg.header); } ;
		void sensorMsgsLaserScanTimeIncrement	(const sensor_msgs::LaserScan::Type& msg)	{data = msg.time_increment;
																							 setTimeStamp(msg.header); } ;
		void sensorMsgsLaserScanScanTime		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.scan_time;
																							 setTimeStamp(msg.header); } ;
		void sensorMsgsLaserScanRangeMin		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.range_min;
																							 setTimeStamp(msg.header); } ;
		void sensorMsgsLaserScanRangeMax		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.range_max;
																							 setTimeStamp(msg.header); } ;
		
		void sensorMsgsJointStatePosition0		(const sensor_msgs::JointState::Type& msg)	{data = msg.position[0];
																							 setTimeStamp(msg.header); } ;

		void setTimeStamp();
		void setTimeStamp(const std_msgs::Header& header);
		void setTimestampFromRosMsgHeader(const std_msgs::Header& header);
		uint64_t timestamp;

		RosNodeDevice* dev;
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		ros::Subscriber subscriber;
		uint32_t subDeviceNumber;
		uint32_t channel;
		double data; 
		std::string msgType;
		std::string topic;
		std::string dataField;
		int queueSize;
		bool callOne;
		bool useEerosSystemTime;
	};
};


extern "C"{
	eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
														double scale, double offset, double rangeMin, double rangeMax, std::string unit,
														std::string additionalArguments);
}


#endif /* ROS_EEROS_ANALOGIN_HPP_ */