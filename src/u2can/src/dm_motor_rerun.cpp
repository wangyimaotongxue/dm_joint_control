#include "damiao.h"
#include "unistd.h"
#include <cmath>


damiao::Motor M1(damiao::DMH3510,0x01, 0x00);
damiao::Motor M2(damiao::DM4310,0x02, 0x00);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);

// // 初始化变量
//     float position = 0.1f;  // 起始位置
//     float increment = 0.01f; // 初始增量
//     bool positive_direction = true; // 运动方向标志
//     const float max_position = 1.0f; // 最大位置
//     const float min_increment = 0.01f; // 最小增量


int main(int argc, char  *argv[])
{

    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    dm = damiao::Motor_Control(serial);

    dm.addMotor(&M1);

    dm.enable(M1);  

    sleep(1);
    if(dm.switchControlMode(M1,damiao::MIT_MODE))
        std::cout << "Switch to MIT_MODE Success" << std::endl;
    
    sleep(1);
    while(1)
    {
        float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
        // 控制电机转动

        dm.control_mit(M1, 20, 0.5, position, 0.0001, 0.1);

        dm.refresh_motor_status(M1);
        // dm.refresh_motor_status(M2);
        std::cout<<"motor1--- POS:"<<M1.Get_Position()<<" VEL:"<<M1.Get_Velocity()<<" CUR:"<<M1.Get_tau()<<std::endl;




        // // 更新下一个目标位置
        // if (positive_direction) {
        //     position += increment;
        //     // 切换到负方向，增加增量
        //     positive_direction = false;
        //     increment += 0.01f; // 每次增加0.01
        // } else {
        //     position -= increment;
        //     // 切换到正方向
        //     positive_direction = true;
        // }
        
        // // 检查是否达到最大位置
        // if (position >= max_position) {
        //     std::cout << "达到目标位置1rad，停止运动" << std::endl;
        //     break; // 退出循环
        // }
        
        // // 确保位置不小于0（可选）
        // if (position < 0) {
        //     position = 0;
        // }
        
        usleep(100000); // 100ms延迟，让电机有时间响应

    }   

    // 在退出前禁用电机
    dm.disable(M1);
    // RCLCPP_INFO(node->get_logger(), "正在关闭电机并退出...");

    // 显式释放资源
    serial.reset();

    return 0;
}

