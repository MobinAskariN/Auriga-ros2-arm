#include "arm_hardware/arm_hardware_interface.hpp"

namespace arm_hardware {

hardware_interface::CallbackReturn ArmHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    left_motor_id_ = std::stoi(info_.hardware_parameters["left_motor_id"]);
    right_motor_id_ = std::stoi(info_.hardware_parameters["right_motor_id"]);
    port_ = info_.hardware_parameters["dynamixel_port"];

    driver_ = std::make_shared<XL330Driver>(port_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() !=0) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // for (const auto & [name, descr] : joint_command_interfaces_)
    // {
    //     RCLCPP_INFO(get_logger(), "COMMAND INTERFACE NAME: ");
    //     RCLCPP_INFO(get_logger(), name.c_str());
    // }
    // for (const auto & [name, descr] : joint_state_interfaces_)
    // {
    //     RCLCPP_INFO(get_logger(), "STATE INTERFACE NAME: ");
    //     RCLCPP_INFO(get_logger(), name.c_str());
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
    left_pos_ = 0.0;
    right_pos_ = 0.0;
    left_vel_ = 0.0;
    right_vel_ = 0.0;

    left_vel_cmd_ = 0.0;
    right_vel_cmd_ = 0.0;

    driver_->activateWithVelocityMode(left_motor_id_);
    driver_->activateWithVelocityMode(right_motor_id_);

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_->deactivate(left_motor_id_);
    driver_->deactivate(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back("base_left_wheel_joint",
                                    hardware_interface::HW_IF_POSITION,
                                    &left_pos_);

    state_interfaces.emplace_back("base_left_wheel_joint",
                                    hardware_interface::HW_IF_VELOCITY,
                                    &left_vel_);

    state_interfaces.emplace_back("base_right_wheel_joint",
                                    hardware_interface::HW_IF_POSITION,
                                    &right_pos_);

    state_interfaces.emplace_back("base_right_wheel_joint",
                                    hardware_interface::HW_IF_VELOCITY,
                                    &right_vel_);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back("base_left_wheel_joint",
                                    hardware_interface::HW_IF_VELOCITY,
                                    &left_vel_cmd_);

    command_interfaces.emplace_back("base_right_wheel_joint",
                                    hardware_interface::HW_IF_VELOCITY,
                                    &right_vel_cmd_);

    return command_interfaces;
}

hardware_interface::return_type ArmHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
    left_vel_ = driver_->getVelocityRadianPerSec(left_motor_id_);
    right_vel_ = -1.0 * driver_->getVelocityRadianPerSec(right_motor_id_);

    if (std::abs(left_vel_) < 0.03) left_vel_ = 0.0;
    if (std::abs(right_vel_) < 0.03) right_vel_ = 0.0;

    left_pos_ += left_vel_ * period.seconds();
    right_pos_ += right_vel_ * period.seconds();

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type ArmHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
    driver_->setTargetVelocityRadianPerSec(left_motor_id_, left_vel_cmd_);
    driver_->setTargetVelocityRadianPerSec(right_motor_id_, -1.0 * right_vel_cmd_);

    return hardware_interface::return_type::OK;
}
} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)