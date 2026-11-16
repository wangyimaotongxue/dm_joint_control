// USB2CANFD libusb backend.
#include "canfd_motor.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dm_canfd_set_zero_position");

  try {
    const auto config = turret_canfd::load_motor_config(*node);
    turret_canfd::MotorSession session(config, false);
    RCLCPP_WARN(
      node->get_logger(), "即将把 CAN ID 0x%03x 的当前位置设为零点", config.can_id);
    session.set_zero_and_save();
    RCLCPP_INFO(node->get_logger(), "零点已设置并保存");
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "CANFD 设置零点失败: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
