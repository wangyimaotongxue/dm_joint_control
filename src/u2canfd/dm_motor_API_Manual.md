# 达妙电机 USB 转 CANFD 接口说明手册（u2canfd）

> 目标：让 `u2canfd` 用起来和 `src/u2can` 串口包一样简单。
> 普通单设备程序不需要写任何 CAN 接收解析，直接调用接口即可。

## 1. 引用什么文件、各做什么

| 文件 | 作用 | 你需要碰吗 |
|------|------|-----------|
| `include/protocol/pub_user.h` | 达妙官方 C SDK 声明（`libdm_device.so`） | 一般不用 |
| `include/protocol/usb_class.h` + `src/protocol/usb_class.cpp` | 打开设备、设波特率、开通道、发帧 | 一般不用 |
| `include/protocol/damiao.h` + `src/protocol/damiao.cpp` | 电机控制层，你要用的 API 都在这 | **用** |
| `src/main.cpp` | 最小 MIT 控制示例 | 照抄 |
| `src/dm_set_zero_position.cpp` | 设置零点示例 | 照抄 |

引用：

```cpp
#include "protocol/damiao.h"
```

## 2. 快速开始

1. 安装 libusb：`sudo apt install libusb-1.0-0-dev`
2. 给设备加 udev 权限：单路 `34b7:6877`，双路 `34b7:6632`
3. 查串号：编译后运行 `./dev_sn`，`SN:` 后面那串
4. 把串号填进 `main.cpp` 的 `SN`，编译运行：

```bash
mkdir -p build && cd build
cmake ..
make
sudo ./dm_main          # 最小 MIT 示例
sudo ./dm_set_zero_position  # 设置零点
```

## 3. 定义控制对象

```cpp
const char*    SN       = "你的SN";
const uint32_t NOM_BAUD = 1000000;  // 仲裁域
const uint32_t DAT_BAUD = 5000000;  // 数据域：CANFD 电机填 5M，经典 CAN 电机填 1M

std::vector<damiao::DmActData> init;
init.push_back(damiao::DmActData{
    .motorType = damiao::DM4310,
    .mode      = damiao::MIT_MODE,
    .can_id    = 0x01,   // 节点 ID
    .mst_id    = 0x11,   // 反馈 ID，建议 can_id + 0x10，不要设 0x00
    .channel   = CHANNEL0});

auto control = std::make_shared<damiao::Motor_Control>(
    DEV_USB2CANFD, NOM_BAUD, DAT_BAUD, SN, &init);  // 双路设备填 DEV_USB2CANFD_DUAL

auto m = control->getMotor(CHANNEL0, 0x01);
if (!m) { /* 电机没注册 */ }
```

接收回调已经由 `Motor_Control` 构造时自动挂好，你不用再写 `device_hook_to_rec`。

## 4. 在线检测

构造成功只说明 USB 盒子打开了，不代表电机在线。先探活：

```cpp
if (!control->wait_for_link(*m, 1000))
{
    std::cerr << "motor not online" << std::endl;
    return 1;
}
```

`wait_for_link` 会反复读寄存器并等回包，超时返回 `false`。

## 5. 使能 / 失能

```cpp
control->enable(*m);    // 单台使能
control->disable(*m);   // 单台失能
control->enable_all();  // 全部使能
control->disable_all(); // 全部失能
```

建议：要改参数/切模式时，先失能，改完再使能。

## 6. 设置零点

失能状态下，把电机摆到你要的零点位置，然后：

```cpp
control->disable(*m);
std::this_thread::sleep_for(std::chrono::milliseconds(100));
control->switchControlMode(*m, damiao::MIT);   // 切到实际使用的模式
std::this_thread::sleep_for(std::chrono::milliseconds(100));

control->set_zero_position(*m);      // 当前位置记为 0
control->save_motor_param(*m);       // 写入 flash，断电不丢
```

## 7. 控制模式

切模式会写寄存器并读回确认，返回 `true` 表示确认成功：

```cpp
control->switchControlMode(*m, damiao::MIT);       // 力位混合
control->switchControlMode(*m, damiao::POS_VEL);   // 位置速度
control->switchControlMode(*m, damiao::VEL);       // 速度
control->switchControlMode(*m, damiao::POS_FORCE); // 位置力矩
```

### MIT 模式

```cpp
control->control_mit(*m, 50.0f, 0.3f, 0.0f, 0.0f, 0.5f);
//                      kp     kd    q     dq    tau
// kp 刚度[0,500], kd 阻尼[0,5], q 位置(rad), dq 速度(rad/s), tau 力矩(Nm)
```

### 位置速度模式

```cpp
control->control_pos_vel(*m, 3.14f, 2.0f);  // 绝对位置 rad, 速度上限 rad/s
```

### 速度模式

```cpp
control->control_vel(*m, 5.0f);  // rad/s
```

## 8. 读状态

达妙是一发一收：只有发控制帧或 `refresh_motor_status` 后，电机对象里的状态才更新。

```cpp
control->refresh_motor_status(*m);
std::this_thread::sleep_for(std::chrono::milliseconds(5));

float pos = m->Get_Position();  // rad
float vel = m->Get_Velocity();  // rad/s
float tau = m->Get_tau();       // Nm
int   err = m->Get_Err();       // 故障码
```

## 9. 参数读写

```cpp
// 读参数（现在会等回包 + 超时）
float uv = control->read_motor_param(*m, damiao::UV_Value);

// 改参数（会写 + 读回确认，返回 true 表示成功）
if (control->change_motor_param(*m, damiao::MAX_SPD, 10.0f))
    std::cout << "change success" << std::endl;

// 保存到 flash（内部会先失能）
control->save_motor_param(*m);
```

寄存器 ID 用 `damiao::DM_REG` 里的枚举（`UV_Value`、`MAX_SPD`、`CTRL_MODE`、`TIMEOUT` 等）。

## 10. 完整最小示例

就是 [src/main.cpp](src/dm_set_zero_position.cpp)，核心流程：

```text
构造 Motor_Control → getMotor → wait_for_link → disable → switchControlMode(MIT) → enable
→ 循环: control_mit + refresh_motor_status + Get_Position → 退出时 disable
```

## 11. 内部实现速览

- `addMotor`：一台电机同时用 `can_id` 和 `mst_id` 两个 key 注册，所以发/收都能查到。
- `control_mit`：把浮点参数定点化（kp/kd 12bit，q 16bit，dq/tau 12bit），按协议拼 8 字节。
- `control_pos_vel` / `control_vel`：直接 memcpy float（小端），发到 `can_id + 模式偏移`。
- `read_motor_param`：发 `0x33`，置 `read_write_save=true`，轮询等回包（20 次 × 10ms），
  超时返回 0。回包由内置回调写进 `param_map`。
- `switchControlMode` / `change_motor_param`：发 `0x55` 写寄存器，等回包，再读回比对。
- `set_zero_position`：发 `0xFE` 到 `can_id + 模式偏移`。
- `save_motor_param`：先发 `0xFD` 失能，再发 `0xAA` 保存。
- `refresh_motor_status`：发 `0xCC` 要一帧反馈。
- 内置回调 `canframeCallback`：区分参数回包（`payload[2]` 为 0x33/0x55/0xAA）和普通反馈，
  普通反馈按 `limit_param` 定点反解成位置/速度/力矩。

## 12. 常见坑

1. 波特率必须和上位机一致：经典 CAN 电机用 1M/1M，CANFD 电机才用 1M/5M。
   库会按 `DAT_BAUD` 自动选择帧类型：`<=1M` 发经典 CAN 2.0，`>1M` 发 CANFD+BRS。
   5M 多电机时末端接 120Ω 终端电阻。
2. `mst_id` 别设 `0x00`，建议 `can_id + 0x10`，且和 `can_id` 不冲突。
3. 改参数/切模式/保存参数请在失能状态下做。
4. 同时控制两个 USB 设备时，需要自己用 `device_hook_to_rec` 分别注册回调
   （单设备不用管）。
5. `read_motor_param` 超时返回 0；电机不在线时 `wait_for_link` 返回 `false`。
