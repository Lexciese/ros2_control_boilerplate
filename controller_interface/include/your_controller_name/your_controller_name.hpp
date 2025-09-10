#ifndef YOUR_CONTROLLER_NAME__YOUR_CONTROLLER_NAME_HPP_
#define YOUR_CONTROLLER_NAME__YOUR_CONTROLLER_NAME_HPP_

#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <chrono>
#include <iostream>
#include <math.h>
#include <fstream>
#include <tuple>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"

#include "your_controller_name_parameters.hpp"

namespace your_controller_name
{

class YourControllerName : public controller_interface::ControllerInterface

{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<custom_param_name::ParamListener> param_listener_;
  custom_param_name::Params params_;

  std::vector<std::string> hardware_names_;
  std::unordered_map<std::string, uint8_t> hardware_map_;
  uint8_t hardware_size_;
};

}  // namespace your_controller_name

#endif  // YOUR_CONTROLLER_NAME__YOUR_CONTROLLER_NAME_HPP_
