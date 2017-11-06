#include "../include/DigOut.hpp"

using namespace halros;

DigOut::DigOut(	std::string id,
				void* libHandle,
				std::string device,
				uint32_t subDeviceNumber,
				uint32_t channel,
				bool inverted,
				std::string additionalArguments
	   ) : 	Output<bool>(id, libHandle),
			inverted(inverted),
			subDeviceNumber(subDeviceNumber),
			channel(channel),
			dev(RosNodeDevice::getDevice(device)),
			rosNodeHandle(dev->getRosNodeHandle()), 
			data(false),
			queueSize(1000),
			callOne(true)
		{
	
	// parsing additionalArguments:
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
		else if	((key=="useSignalInTimestamp") | (key==" useSignalInTimestamp")) {
			if		(value=="true")		useSignalInTimestamp = true;
			else if	(value=="false")	useSignalInTimestamp = false;
			else std::cout << errorString << "ros-eeros wrapper library: value '" << value << "' for key '" << key << "' is not supported." << std::endl;
		}
		else
			std::cout << errorString << "ros-eeros wrapper library: key '" << key << "' is not supported." << std::endl;
	}
	
	// selecting callback function for ros
	if		( msgType == "sensor_msgs::BatteryState" ) {
		publisher = rosNodeHandle->advertise<sensor_msgs::BatteryState>(topic, queueSize);
		if ( dataField == "present" )
			setFunction = &sensorMsgsBatteryStatePresent;
		else 
			std::cout << errorString << "ros-eeros wrapper library: dataField '" << dataField << "' of msgType '" << msgType << "' is not supported." << std::endl;
	}
	else if ( msgType == "" )
		std::cout << errorString << "ros-eeros wrapper library: msgType is empty." << msgType << std::endl;
	else 
		std::cout << errorString << "ros-eeros wrapper library: msgType '" << msgType << "' is not defined" << std::endl;
}

// set functions for ROS
// /////////////////////
// sensor_msgs::BatteryState
void DigOut::sensorMsgsBatteryStatePresent(const bool value, const uint64_t timestamp, const ros::Publisher& publisher){
	sensor_msgs::BatteryState msg;
	msg.header.stamp = eeros::control::rosTools::convertToRosTime(timestamp);
	msg.present = static_cast<uint8_t>( value );
	publisher.publish(msg);
}



// HAL functions
// /////////////
bool DigOut::get() {
	if(inverted) return !data;
	return data;
}

void DigOut::set(bool value) {
// 	std::cout << "DigOut set: " << value << std::endl;
	if(inverted) value = !value;
	data = value;
	
	uint64_t timestamp;
	if (useSignalInTimestamp)	timestamp = timestampSignalIn;
	else 						timestamp = eeros::System::getTimeNs();
	
	setFunction(value, timestamp, publisher);
}

void DigOut::setTimestampSignalIn(uint64_t timestampNs)
{
	this->timestampSignalIn = timestampNs;
}

extern "C" eeros::hal::Output<bool> *createDigOut(	std::string id,
													void* libHandle,
													std::string device,
													uint32_t subDeviceNumber,
													uint32_t channel,
													bool inverted,
													std::string additionalArguments){
	return new halros::DigOut(id, libHandle, device, subDeviceNumber, channel, inverted, additionalArguments);
}
