// USB2CANFD libusb backend.
#include "canfd_motor.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dm_canfd_motor_run");

  try {
    const auto config = turret_canfd::load_motor_config(*node);
    const double loop_rate_hz =
      turret_canfd::positive_parameter(*node, "loop_rate_hz", 1000.0);
    const double kp = node->declare_parameter<double>("kp", 50.0);
    const double kd = node->declare_parameter<double>("kd", 0.3);
    const double target_position = node->declare_parameter<double>("target_position", 0.0);
    const double target_velocity = node->declare_parameter<double>("target_velocity", 0.1);
    const double target_torque = node->declare_parameter<double>("target_torque", 0.5);
    const bool command_enabled = node->declare_parameter<bool>("command_enabled", false);

    turret_canfd::MotorSession session(config, command_enabled);
    RCLCPP_INFO(
      node->get_logger(), "CANFD 电机初始化完成，CAN ID: 0x%03x", config.can_id);
    if (!command_enabled) {
      RCLCPP_WARN(
        node->get_logger(),
        "command_enabled=false：当前仅刷新反馈，不使能电机、不发送 MIT 控制指令");
    }

    rclcpp::WallRate rate(loop_rate_hz);
    while (rclcpp::ok()) {
      if (command_enabled) {
        session.control().control_mit(
          session.motor(), static_cast<float>(kp), static_cast<float>(kd),
          static_cast<float>(target_position), static_cast<float>(target_velocity),
          static_cast<float>(target_torque));
      }
      session.control().refresh_motor_status(session.motor());

      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "motor--- POS: %.3f VEL: %.3f TAU: %.3f",
        session.motor().Get_Position(), session.motor().Get_Velocity(),
        session.motor().Get_tau());
      rate.sleep();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "CANFD 节点启动失败: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
