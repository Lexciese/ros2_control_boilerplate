#include "your_controller_name/your_controller_name.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "controller_interface/helpers.hpp"

using namespace std::chrono_literals;

namespace your_controller_name {
controller_interface::CallbackReturn YourControllerName::on_init() {
  try {
    node_ = get_node();
    param_listener_ =
        std::make_shared<custom_param_name::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn YourControllerName::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  // Handle controller configuration

  hardware_names_ = {"joint1", "joint2", "joint3"};  // Example hardware names
  hardware_size_ = hardware_names_.size();
  // Map joint names to indices in unordered_map for easier access
  for (uint8_t i = 0; i < hardware_names_.size(); i++) {
    hardware_map_[hardware_names_[i]] = i;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn YourControllerName::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  // Handle controller restarts and dynamic parameter updating

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn YourControllerName::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn YourControllerName::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  // Callback function for cleanup transition
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn YourControllerName::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
  // Callback function for shutdown transition

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn YourControllerName::on_error(
    const rclcpp_lifecycle::State& previous_state) {
  // Callback function for erroneous transition

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
YourControllerName::command_interface_configuration() const {
  // Set to ALL to send command to all actuators
  // Alternatively, use INDIVIDUAL and specify the names
  // Used for sending command to hardware_interface
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& hardware_name : hardware_names_) {
    // We control the actuator with map
    config.names.push_back(hardware_name + "/" +
                           hardware_interface::HW_IF_POSITION);
  }

  return config;
}

controller_interface::InterfaceConfiguration
YourControllerName::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  // Set to ALL to receive all state interfaces
  // Alternatively, use INDIVIDUAL and specify the names
  // Used for getting the sensor data state from hardware_interface
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::ALL;

  return state_interfaces_config;
}

controller_interface::return_type YourControllerName::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  double example_command = 1.0;  // Example command value
  for (int i = 0; i < hardware_size_; i++) {
    bool success = command_interfaces_[hardware_map_["joint1"]].set_value(example_command);
    if (!success) {
      fprintf(stderr, "Failed to set value for joint1 command interface\n");
    }
  }

  double example_state = state_interfaces_[hardware_map_["joint2"]].get_value();

  return controller_interface::return_type::OK;
}

}  // namespace your_controller_name
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(your_controller_name::YourControllerName,
                       controller_interface::ControllerInterface)