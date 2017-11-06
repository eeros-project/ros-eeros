#include "../include/RosNodeDevice.hpp"
#include <eeros/core/Fault.hpp>

using namespace halros;

std::map<std::string, RosNodeDevice *> RosNodeDevice::devices;
// RosNodeDevice* RosNodeDevice::instance = new RosNodeDevice("testNodeInstance");

RosNodeDevice::RosNodeDevice(std::__cxx11::string rosNode) {
	char* dummy_args[] = {NULL};
	int dummy_argc = sizeof(dummy_args)/sizeof(dummy_args[0]) - 1;
	ros::init(dummy_argc, dummy_args, rosNode);
// 	n.g
//  	rosNodeHandle = &n;
	ros::NodeHandle n;
	rosNodeHandle = std::make_shared<ros::NodeHandle>( n );
	
	devices[rosNode] = this;
}

RosNodeDevice::~RosNodeDevice(){
	devices.clear();
}

std::shared_ptr<ros::NodeHandle> RosNodeDevice::getRosNodeHandle() {
	return rosNodeHandle;
}
// 

RosNodeDevice* RosNodeDevice::getDevice(std::__cxx11::string rosNode) {
	auto devIt = devices.find(rosNode);
	if(devIt != devices.end()) {
		return devIt->second;
	}
	else{
		return new RosNodeDevice(rosNode);
	}

}
