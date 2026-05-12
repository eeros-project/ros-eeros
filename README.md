# ros-eeros
The [EEROS Robotics Framework](https://github.com/eeros-project/eeros-framework) uses various [hardware libraries](http://wiki.eeros.org/eeros_architecture/hal/hardware_libraries) to access the underlying hardware. This wrapper library enables the usage of [ROS](http://www.ros.org/). You can use this hardware wrapper library for ROS1 and ROS2. However, please make sure to use release v1.0.1 with ROS1 and release v.2.0.3 with ROS2.

## Documentation
- About ROS: http://www.ros.org/
- About the wrapper library: http://wiki.eeros.org/eeros_architecture/hal/hardware_libraries#ros
- How to install: http://wiki.eeros.org/getting_started/install_wrapper#ros

## Getting Started
Check the following pages for the use of the wrapper library.
- [Interfacing with ROS](https://wiki.eeros.org/getting_started/ros)

The EEROS [Hardware Abstraction Layer](http://wiki.eeros.org/eeros_architecture/hal/start) needs a [configuration file](http://wiki.eeros.org/eeros_architecture/hal/configuration_file) which describes the hardware. A sample hardware configuration file for this wrapper library can be found at
- https://github.com/eeros-project/eeros-framework/blob/master/examples/ros2/RosTest2Config.json

## How to contribute to flink-eeros

The EEROS team would love to accept your contributions! The development on the EEROS Framework is done with the work flow “**develop with a fork**”. So please fork the repository, develop and test your code changes. For code quality, please follow the guidelines put together [here](http://wiki.eeros.org/for_developers/start). In general, the code should adheres to the existing style in the project. Once the changes are ready, a pull request is submitted. Each logical change should be submitted separately to ensure that the history will be understandable.
