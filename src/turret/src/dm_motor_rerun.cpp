#include "rclcpp/rclcpp.hpp"
#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <signal.h>


damiao::Motor M1(damiao::DMH3510, 0x01, 0x11);
damiao::Motor M2(damiao::DM4310, 0x02, 0x12);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

float pos = 1.0f;  // 起始位置
int cont = 0;
volatile sig_atomic_t stop = 0;

void signal_handler(int signum) {
    stop = 1;
}

int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个dm_motor_rerun的节点*/
    auto node = std::make_shared<rclcpp::Node>("dm_motor_rerun");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "dm_motor_rerun.");
     // 注册信号处理，方便Ctrl+C退出
    signal(SIGINT, signal_handler);
    
    RCLCPP_INFO(node->get_logger(), "开始初始化..." );

    try {
        // 1. 初始化串口
        serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        RCLCPP_INFO(node->get_logger(), "串口初始化成功" );
        
        // 2. 初始化电机控制
        dm = damiao::Motor_Control(serial);
        RCLCPP_INFO(node->get_logger(), "电机控制初始化成功" );

        // 3. 添加电机
        dm.addMotor(&M1);
        RCLCPP_INFO(node->get_logger(), "电机添加成功" );

        // 4. 使能电机
        RCLCPP_INFO(node->get_logger(), "正在使能电机..." );
        dm.enable(M1);
        RCLCPP_INFO(node->get_logger(), "电机使能成功" );

        sleep(1);

        // 5. 切换控制模式
        RCLCPP_INFO(node->get_logger(), "正在切换控制模式..." );
        if(dm.switchControlMode(M1, damiao::MIT_MODE)) {
            RCLCPP_INFO(node->get_logger(), "切换到MIT_MODE成功" );
        } else {
            RCLCPP_INFO(node->get_logger(), "切换到MIT_MODE失败" );
            return -1;
        }
        
        sleep(1);

        RCLCPP_INFO(node->get_logger(), "开始控制循环..." );

        // 6. 控制循环
        while(!stop)
        {
            cont++;
            
            // 更温和的位置变化
            if (cont >= 10) {  // 每5秒改变一次方向 (10 * 100ms = 1s)
                cont = 0;
                pos *= -1;
                RCLCPP_INFO(node->get_logger(), "改变方向，新目标位置: %0.3f", pos );
            }
            
            // 控制电机转动 - 使用保守的参数
            dm.control_mit(M1, 10, 1.0, pos, 0.0, 0.3);
            
            // 刷新电机状态 - 直接调用，不检查返回值
            dm.refresh_motor_status(M1);
            
            RCLCPP_INFO(node->get_logger(), "motor1--- POS: %0.3f VEL: %0.3f CUR: %0.3f Target: %0.3f Count: %d", M1.Get_Position(), M1.Get_Velocity(), M1.Get_tau(), pos, cont );

            usleep(100000); // 100ms延迟
        }
    }
    catch (const std::exception& e) {
        RCLCPP_INFO(node->get_logger(), "发生异常: %s", e.what() );
    }

    // 清理
    RCLCPP_INFO(node->get_logger(), "正在关闭电机..." );
    dm.disable(M1);
    serial.reset();
    RCLCPP_INFO(node->get_logger(), "程序退出" );

    return 0;
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
}


