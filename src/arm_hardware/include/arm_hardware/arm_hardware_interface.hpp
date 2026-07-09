#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "arm_hardware/arm_driver.hpp"

namespace arm_hardware {

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    std::shared_ptr<XL330Driver> driver_;

    int left_motor_id_;
    int right_motor_id_;
    std::string port_;

    // ---- State storage ----
    double left_pos_ = 0.0;
    double right_pos_ = 0.0;
    double left_vel_ = 0.0;
    double right_vel_ = 0.0;

    // ---- Command storage ----
    double left_vel_cmd_ = 0.0;
    double right_vel_cmd_ = 0.0;
};

}  // namespace arm_hardware

#endif