#ifndef TURRET_CANFD_MOTOR_HPP_
#define TURRET_CANFD_MOTOR_HPP_

#include "protocol/damiao.h"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace turret_canfd
{

struct MotorConfig
{
  std::string device_sn;
  uint32_t nominal_baud;
  uint32_t data_baud;
  damiao::DM_Motor_Type motor_type;
  uint16_t can_id;
  uint16_t master_id;
};

inline damiao::DM_Motor_Type parse_motor_type(const std::string & name)
{
  static const std::unordered_map<std::string, damiao::DM_Motor_Type> motor_types = {
    {"DM3507", damiao::DM3507},
    {"DM4310", damiao::DM4310},
    {"DM4310_48V", damiao::DM4310_48V},
    {"DM4340", damiao::DM4340},
    {"DM4340_48V", damiao::DM4340_48V},
    {"DM6006", damiao::DM6006},
    {"DM6248", damiao::DM6248},
    {"DM8006", damiao::DM8006},
    {"DM8009", damiao::DM8009},
    {"DM10010L", damiao::DM10010L},
    {"DM10010", damiao::DM10010},
    {"DMH3510", damiao::DMH3510},
    {"DMH6215", damiao::DMH6215},
    {"DMS3519", damiao::DMS3519},
    {"DMG6220", damiao::DMG6220},
  };

  const auto motor_type = motor_types.find(name);
  if (motor_type == motor_types.end()) {
    throw std::invalid_argument("不支持的 motor_type: " + name);
  }
  return motor_type->second;
}

inline MotorConfig load_motor_config(rclcpp::Node & node)
{
  const auto device_sn = node.declare_parameter<std::string>("device_sn", "");
  const auto nominal_baud = node.declare_parameter<int64_t>("nominal_baud", 1000000);
  const auto data_baud = node.declare_parameter<int64_t>("data_baud", 5000000);
  const auto motor_type_name = node.declare_parameter<std::string>("motor_type", "DMH3510");
  const auto can_id = node.declare_parameter<int64_t>("can_id", 0x01);
  const auto master_id = node.declare_parameter<int64_t>("master_id", 0x11);

  if (device_sn.empty()) {
    throw std::invalid_argument(
            "缺少 device_sn 参数；请先运行 ros2 run turret dm_canfd_dev_sn 查询");
  }
  if (nominal_baud <= 0 || nominal_baud > UINT32_MAX ||
    data_baud <= 0 || data_baud > UINT32_MAX)
  {
    throw std::invalid_argument("nominal_baud 和 data_baud 必须是有效的正整数");
  }
  if (can_id < 0 || can_id > 0x7ff || master_id < 0 || master_id > 0x7ff) {
    throw std::invalid_argument("can_id 和 master_id 必须是 0~0x7ff 的标准 CAN ID");
  }
  if (can_id == master_id) {
    throw std::invalid_argument("can_id 和 master_id 不能相同");
  }

  return MotorConfig{
    device_sn,
    static_cast<uint32_t>(nominal_baud),
    static_cast<uint32_t>(data_baud),
    parse_motor_type(motor_type_name),
    static_cast<uint16_t>(can_id),
    static_cast<uint16_t>(master_id)};
}

inline double positive_parameter(
  rclcpp::Node & node, const std::string & name, double default_value)
{
  const double value = node.declare_parameter<double>(name, default_value);
  if (value <= 0.0) {
    throw std::invalid_argument(name + " 必须大于 0");
  }
  return value;
}

class MotorSession
{
public:
  explicit MotorSession(const MotorConfig & config, bool enable_motor = true)
  : config_(config)
  {
    motor_data_.push_back(
      damiao::DmActData{config.motor_type, damiao::MIT_MODE, config.can_id, config.master_id});
    control_ = std::make_unique<damiao::Motor_Control>(
      config.nominal_baud, config.data_baud, config.device_sn, &motor_data_);
    motor_ = control_->getMotor(config.can_id);
    if (!motor_) {
      throw std::runtime_error("CANFD SDK 未注册配置的电机");
    }

    // 厂商 CANFD SDK 的构造函数会默认切换到速度模式。这里先失能，再明确
    // 切回云台节点使用的 MIT 模式，最后按 MIT 模式 CAN ID 重新使能。
    control_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!control_->switchControlMode(*motor_, damiao::MIT)) {
      throw std::runtime_error("切换 MIT 模式失败");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (enable_motor) {
      control_->control_cmd(config.can_id + damiao::MIT_MODE, 0xFC);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  ~MotorSession()
  {
    if (control_) {
      // 使用明确的 MIT 模式 ID 失能，避免异步参数回包尚未更新本地模式。
      control_->control_cmd(config_.can_id + damiao::MIT_MODE, 0xFD);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  MotorSession(const MotorSession &) = delete;
  MotorSession & operator=(const MotorSession &) = delete;

  damiao::Motor & motor() {return *motor_;}
  damiao::Motor_Control & control() {return *control_;}

  void set_zero_and_save()
  {
    control_->control_cmd(config_.can_id + damiao::MIT_MODE, 0xFD);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    control_->control_cmd(config_.can_id + damiao::MIT_MODE, 0xFE);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    control_->save_motor_param(*motor_);
  }

private:
  MotorConfig config_;
  std::vector<damiao::DmActData> motor_data_;
  std::unique_ptr<damiao::Motor_Control> control_;
  std::shared_ptr<damiao::Motor> motor_;
};

}  // namespace turret_canfd

#endif  // TURRET_CANFD_MOTOR_HPP_
