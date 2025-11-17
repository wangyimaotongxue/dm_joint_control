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

#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <signal.h>

damiao::Motor M1(damiao::DMH3510, 0x01, 0x00);
damiao::Motor M2(damiao::DM4310, 0x02, 0x00);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

float pos = 0.1f;  // 起始位置
int cont = 0;
volatile sig_atomic_t stop = 0;

void signal_handler(int signum) {
    stop = 1;
}

int main(int argc, char *argv[])
{
    // 注册信号处理，方便Ctrl+C退出
    signal(SIGINT, signal_handler);
    
    std::cout << "开始初始化..." << std::endl;

    try {
        // 1. 初始化串口
        serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        std::cout << "串口初始化成功" << std::endl;
        
        // 2. 初始化电机控制
        dm = damiao::Motor_Control(serial);
        std::cout << "电机控制初始化成功" << std::endl;

        // 3. 添加电机
        dm.addMotor(&M1);
        std::cout << "电机添加成功" << std::endl;

        // 4. 使能电机
        std::cout << "正在使能电机..." << std::endl;
        dm.enable(M1);
        std::cout << "电机使能成功" << std::endl;

        sleep(1);

        // 5. 切换控制模式
        std::cout << "正在切换控制模式..." << std::endl;
        if(dm.switchControlMode(M1, damiao::MIT_MODE)) {
            std::cout << "切换到MIT_MODE成功" << std::endl;
        } else {
            std::cout << "切换到MIT_MODE失败" << std::endl;
            return -1;
        }
        
        sleep(1);

        std::cout << "开始控制循环..." << std::endl;

        // 6. 控制循环
        while(!stop)
        {
            cont++;
            
            // 更温和的位置变化
            if (cont >= 50) {  // 每5秒改变一次方向 (50 * 100ms = 5s)
                cont = 0;
                pos *= -1;
                std::cout << "改变方向，新目标位置: " << pos << std::endl;
            }
            
            // 控制电机转动 - 使用保守的参数
            dm.control_mit(M1, 10, 1.0, pos, 0.0, 0.3);
            
            // 刷新电机状态 - 直接调用，不检查返回值
            dm.refresh_motor_status(M1);
            
            std::cout << "motor1--- POS:" << M1.Get_Position() 
                     << " VEL:" << M1.Get_Velocity() 
                     << " CUR:" << M1.Get_tau() 
                     << " Target:" << pos 
                     << " Count:" << cont 
                     << std::endl;

            usleep(100000); // 100ms延迟
        }
    }
    catch (const std::exception& e) {
        std::cout << "发生异常: " << e.what() << std::endl;
    }

    // 清理
    std::cout << "正在关闭电机..." << std::endl;
    dm.disable(M1);
    serial.reset();
    std::cout << "程序退出" << std::endl;

    return 0;
}