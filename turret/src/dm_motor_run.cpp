#include "rclcpp/rclcpp.hpp"
#include "damiao.h"
#include "unistd.h"
#include <cmath>

damiao::Motor M1(damiao::DMH3510,0x01, 0x00);
damiao::Motor M2(damiao::DM4310,0x02, 0x00);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个dm_motor_run的节点*/
    auto node = std::make_shared<rclcpp::Node>("dm_motor_run");


    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "dm_motor_run节点已经启动.");
    /* 运行节点，并检测退出信号 Ctrl+C*/

    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    dm = damiao::Motor_Control(serial);

    dm.addMotor(&M1);
    // dm.addMotor(&M2);
    // 使能电机
    dm.enable(M1);  
    // dm.disable(M1);
    // dm.disable(M2);
    sleep(1);
    if(dm.switchControlMode(M1,damiao::MIT_MODE))
        RCLCPP_INFO(node->get_logger(), "Switch to MIT_MODE Success" );

    
    sleep(1);
    while(rclcpp::ok())
    {
        float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
        // 控制电机转动
        dm.control_mit(M1, 50, 0.3, 0, 0.1, 0.5);
        // dm.control_mit(M1, 30, 0.3, q*10, 0, 0);
        // dm.control_vel(M1, q*100);
        // dm.control_pos_vel(M2, q*10,10);
        dm.refresh_motor_status(M1);
        // dm.refresh_motor_status(M2);
        // std::cout<<"motor1--- POS:"<<M1.Get_Position()<<" VEL:"<<M1.Get_Velocity()<<" CUR:"<<M1.Get_tau()<<std::endl;
        RCLCPP_INFO(node->get_logger(), "motor1--- POS: %.3f VEL: %.3f CUR: %.3f", M1.Get_Position(), M1.Get_Velocity(), M1.Get_tau());
        // std::cout<<"motor2--- POS:"<<M2.Get_Position()<<" VEL:"<<M2.Get_Velocity()<<" CUR:"<<M2.Get_tau()<<std::endl;
        usleep(1000);
        // std::cout<<"motor1 pos:"<<M1.Get_Position()<<std::endl;

        // 处理ROS事件，让Ctrl+C能够被捕获
        // rclcpp::spin(node);
        
    }   

    // 在退出前禁用电机
    dm.disable(M1);
    RCLCPP_INFO(node->get_logger(), "正在关闭电机并退出...");

    // 显式释放资源
    serial.reset();
    
    // 停止运行
    rclcpp::shutdown();
    
    return 0;

}
