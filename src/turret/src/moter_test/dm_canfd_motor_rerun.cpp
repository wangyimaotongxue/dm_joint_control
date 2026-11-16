// USB2CANFD libusb backend.
#include "canfd_motor.hpp"

#include <chrono>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dm_canfd_motor_rerun");

  try {
    const auto config = turret_canfd::load_motor_config(*node);
    const double loop_rate_hz =
      turret_canfd::positive_parameter(*node, "loop_rate_hz", 10.0);
    const double switch_period =
      turret_canfd::positive_parameter(*node, "switch_period", 1.0);
    const double position_amplitude =
      turret_canfd::positive_parameter(*node, "position_amplitude", 1.0);
    const double kp = node->declare_parameter<double>("kp", 10.0);
    const double kd = node->declare_parameter<double>("kd", 1.0);
    const double target_velocity = node->declare_parameter<double>("target_velocity", 0.0);
    const double target_torque = node->declare_parameter<double>("target_torque", 0.3);

    turret_canfd::MotorSession session(config);
    double target_position = position_amplitude;
    auto next_switch = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(switch_period));

    RCLCPP_INFO(
      node->get_logger(), "CANFD 往复控制已启动，CAN ID: 0x%03x", config.can_id);
    rclcpp::WallRate rate(loop_rate_hz);
    while (rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_switch) {
        target_position = -target_position;
        next_switch = now +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(switch_period));
        RCLCPP_INFO(node->get_logger(), "新的目标位置: %.3f", target_position);
      }

      session.control().control_mit(
        session.motor(), static_cast<float>(kp), static_cast<float>(kd),
        static_cast<float>(target_position), static_cast<float>(target_velocity),
        static_cast<float>(target_torque));
      session.control().refresh_motor_status(session.motor());
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "motor--- POS: %.3f VEL: %.3f TAU: %.3f TARGET: %.3f",
        session.motor().Get_Position(), session.motor().Get_Velocity(),
        session.motor().Get_tau(), target_position);
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
