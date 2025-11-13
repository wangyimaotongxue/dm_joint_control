#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个dm_motor_rerun的节点*/
    auto node = std::make_shared<rclcpp::Node>("dm_motor_rerun");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "dm_motor_rerun.");
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
}
