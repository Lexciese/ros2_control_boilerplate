#include "your_hardware_interface/your_hardware_interface.hpp"

#include <limits>
#include <memory>
#include <vector>

namespace your_hardware_interface {

YourHardwareInterface::YourHardwareInterface()
    : rclcpp::Node("YourHardwareInterface") {}
YourHardwareInterface::~YourHardwareInterface() {}

hardware_interface::CallbackReturn YourHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YourHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("YourHardwareInterface"),
              "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
YourHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces_conf;

  /*
   EXAMPLE: Button State Interface
   state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
       info_.sensors[1].name, info_.sensors[1].state_interfaces[0].name,
       &button_msg_));
  */

  return state_interfaces_conf;
}

hardware_interface::CallbackReturn YourHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YourHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YourHardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_ERROR(rclcpp::get_logger("YourHardwareInterface"),
               "Hardware entered error state, attempting to recover");

  return hardware_interface::CallbackReturn::ERROR;
}
hardware_interface::return_type YourHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type YourHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

}  // namespace your_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(your_hardware_interface::YourHardwareInterface,
                       hardware_interface::SystemInterface)
