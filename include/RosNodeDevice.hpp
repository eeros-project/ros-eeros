#pragma once

#include <eeros/control/ros2/RosTools.hpp>
#include <map>
#include <memory>
#include <string>

using namespace eeros::control;

namespace roseeros {

inline constexpr std::string_view errorString = "\033[1;31mERROR ros-eeros: \033[0m";

inline void signalHandler1(int signo) {
    RosTools::cleanup();         // shuts down rclcpp
}

/**
 * @brief Manages named ROS2 nodes, ensuring one node per unique name.
 *
 * Implements a per-name singleton pattern: calling getDevice() with the
 * same name always returns the same instance. The underlying ROS2 node
 * is initialized on first access and cleaned up on destruction.
 *
 * @note Non-copyable and non-movable.
 *
 * @example
 * @code
 * auto& device = RosNodeDevice::getDevice("my_node");
 * auto node = device.getRosNodeHandle();
 * @endcode
 */
class RosNodeDevice {
 public:
  /**
   * @brief Destructor. Cleans up all ROS2 resources and clears the device registry.
   */
  virtual ~RosNodeDevice() {
    rclcpp::shutdown();
    devices.clear();
  }

  // Non-copyable, non-movable (singleton-per-name pattern)
  RosNodeDevice(const RosNodeDevice&)            = delete;
  RosNodeDevice& operator=(const RosNodeDevice&) = delete;
  RosNodeDevice(RosNodeDevice&&)                 = delete;
  RosNodeDevice& operator=(RosNodeDevice&&)      = delete;

  /**
   * @brief Returns the device instance for the given name, creating it if necessary.
   *
   * If a device with the given name already exists in the registry, the existing
   * instance is returned. Otherwise, a new ROS2 node is created and registered.
   *
   * @param name Unique name for the ROS2 node.
   * @return Reference to the RosNodeDevice instance for the given name.
   */
  [[nodiscard]] static RosNodeDevice& getDevice(const std::string& name) {
    if (auto it = devices.find(name); it != devices.end()) {
      return *it->second;
    }
    auto& ptr = devices[name];
    ptr.reset(new RosNodeDevice(name));
    return *ptr;
  }

  /**
   * @brief Returns the underlying ROS2 node handle.
   * @return Shared pointer to the rclcpp::Node instance.
   */
  [[nodiscard]] rclcpp::Node::SharedPtr getRosNodeHandle() const {
    return node;
  }
 
  /**
   * @brief Returns the name of the ROS2 node.
   * @return The node name as given to getDevice().
   */
  [[nodiscard]] const std::string& getName() const {
    return name;
  }

 private:
  /**
   * @brief Constructs a new RosNodeDevice, initializing ROS2 and creating the node.
   * @param name Unique name for the ROS2 node.
   */
  explicit RosNodeDevice(const std::string& name) : name(name) {
    std::cout << "Creating ROS node device: '" << name << std::endl;
    if (!rclcpp::ok()) {RosTools::initRos(0, nullptr);
    RosTools::registerCustomSigIntHandler(signalHandler1);}
    node = RosTools::initNode(name);
  }

  std::string name;
  rclcpp::Node::SharedPtr node;
  /// @brief Registry of all active RosNodeDevice instances, keyed by node name.
  static inline std::map<std::string, std::unique_ptr<RosNodeDevice>> devices;
};

}
