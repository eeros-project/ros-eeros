#include <iostream>
#include "../include/AnalogIn.hpp"
#include <ros/callback_queue.h>

using namespace halros;

AnalogIn::AnalogIn(std::string id,
					 void* libHandle,
					 std::string device,		//ROS node name
					 uint32_t subDeviceNumber,
					 uint32_t channel,
					 double scale,
					 double offset,
					 double rangeMin,
					 double rangeMax,
					 std::string unit,
					 std::string additionalArguments
		  ) : ScalableInput<double>(id, libHandle, scale, offset, rangeMin, rangeMax, unit),
		      subDeviceNumber(subDeviceNumber),
		      channel(channel),
		      dev(RosNodeDevice::getDevice(device)),
		      rosNodeHandle(dev->getRosNodeHandle()),
		      data(0),
		      queueSize(1000),
		      callOne(true),
		      useEerosSystemTime(false)
		      {
	// parsing additionalArguments:
	// ////////////////////////////
	auto s = additionalArguments;
	bool stop = false;
	while(!stop) {
		if(s.find(";") == std::string::npos) stop=true;
		std::string statement = s.substr(0, s.find(";"));
		std::string key = statement.substr(0, statement.find("="));
		std::string value = statement.substr(statement.find("=")+1);
		s = s.substr(s.find(";")+1);
		
		if		((key=="msgType") | (key==" msgType")) 
			msgType = value;
		else if	((key=="topic") | (key==" topic")) 
			topic = value;
		else if	((key=="dataField") | (key==" dataField")) 
			dataField = value;
		else if	((key=="queueSize") | (key==" queueSize"))
			queueSize = std::stoi(value);
		else if	((key=="useEerosSystemTime") | (key==" useEerosSystemTime")) {
			if		(value=="true")		useEerosSystemTime = true;
			else if	(value=="false")	useEerosSystemTime = false;
			else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
		}
		else if	((key=="callOne") | (key==" callOne")) {
			if		(value=="true")		callOne = true;
			else if	(value=="false")	callOne = false;
			else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
		}
		else
			std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
	}
	
	// 3.) Extend parser by selecting correct callback function for ros
	// ////////////////////////////////////////////////////////////////
	if		( msgType == "std_msgs::Float64" ) {
			subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::stdMsgsFloat64Data, this);
	}
	
	else if ( msgType == "sensor_msgs::LaserScan" ) {
		if 		( dataField == "angle_min" )
					subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::sensorMsgsLaserScanAngleMin, this);
		else if ( dataField == "angle_max" )
					subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::sensorMsgsLaserScanAngleMax, this);
		else if ( dataField == "angle_increment" )
					subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::sensorMsgsLaserScanAngleIncrement, this);
		else if ( dataField == "time_increment" )
					subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::sensorMsgsLaserScanTimeIncrement, this);
		else if ( dataField == "scan_time" )
					subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::sensorMsgsLaserScanScanTime, this);
		else if ( dataField == "range_min" )
					subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::sensorMsgsLaserScanRangeMin, this);
		else if ( dataField == "range_max" )
					subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::sensorMsgsLaserScanRangeMax, this);
		else
			std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
	}
	
	else if	( msgType == "sensor_msgs::JointState" ) {
		if 		( dataField == "position0" )
					subscriber = rosNodeHandle->subscribe(topic, queueSize, &AnalogIn::sensorMsgsJointStatePosition0, this);
		else
			std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
	}
	
	else if ( msgType == "" )
		std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
	else 
		std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}


// HAL functions
// /////////////
double AnalogIn::get() {
	if ( callOne )		
		ros::getGlobalCallbackQueue()->callOne();			// calls callback fct. only for the oldest message
	else
		ros::getGlobalCallbackQueue()->callAvailable();		// calls callback fct. for all available messages.
															// Only newest message is processed. Older ones are discarded.	
	
	double inVal = (data - offset) / scale;
	if(inVal > maxIn) inVal = maxIn;
	if(inVal < minIn) inVal = minIn;
	
// 	std::cout << "AnalogIn received Timestamp = " << timestamp << " at ros::Time::now().toNSec() = " << ros::Time::now().toNSec() << std::endl;
	
	return inVal;
}

uint64_t AnalogIn::getTimestamp() {
	return timestamp;
}

void AnalogIn::setTimeStamp()
{
	timestamp = eeros::System::getTimeNs();
}

void AnalogIn::setTimeStamp(const std_msgs::Header& header)
{
	if (useEerosSystemTime)	setTimeStamp();
	else					setTimestampFromRosMsgHeader(header);
	
}

void AnalogIn::setTimestampFromRosMsgHeader(const std_msgs::Header& header) {
	timestamp = header.stamp.toNSec();
}


extern "C"{
	eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, 
							void* libHandle, 
							std::string device, 
							uint32_t subDeviceNumber, 
							uint32_t channel, 
							double scale, 
							double offset, 
							double rangeMin, 
							double rangeMax, 
							std::string unit,
							std::string additionalArguments){
		return new halros::AnalogIn(id, libHandle, device, subDeviceNumber, channel, scale, offset, rangeMin, rangeMax, unit, additionalArguments);
	}
}

