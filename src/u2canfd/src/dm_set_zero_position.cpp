#include "protocol/damiao.h"

#include <csignal>
#include <iostream>
#include <thread>

void on_signal(int)
{
}

int main()
{
    std::signal(SIGINT, on_signal);

    const char*    SN       = "05E09A89546004EBDA204DB282566246"; // 替换成你的 SN
    const uint32_t NOM_BAUD = 1000000; // 仲裁域
    const uint32_t DAT_BAUD = 5000000; // 数据域：CANFD 电机填 5M，经典 CAN 电机填 1M

    std::vector<damiao::DmActData> init;
    init.push_back(damiao::DmActData{
        .motorType = damiao::DM4310,
        .mode      = damiao::MIT_MODE,
        .can_id    = 0x01,
        .mst_id    = 0x11,
        .channel   = CHANNEL0});

    auto control = std::make_shared<damiao::Motor_Control>(
        DEV_USB2CANFD, NOM_BAUD, DAT_BAUD, SN, &init);

    auto m = control->getMotor(CHANNEL0, 0x01);
    if (!m)
    {
        std::cerr << "motor not registered" << std::endl;
        return 1;
    }

    if (!control->wait_for_link(*m, 1000))
    {
        std::cerr << "motor not online, check power / wiring / id / baud" << std::endl;
        return 1;
    }

    // 失能状态下操作
    control->disable(*m);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!control->switchControlMode(*m, damiao::MIT))
    {
        std::cerr << "switch to MIT failed" << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 把当前位置记为 0
    control->set_zero_position(*m);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 写入 flash，断电不丢
    control->save_motor_param(*m);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "zero position set and saved, motor remains disabled" << std::endl;
    return 0;
}
