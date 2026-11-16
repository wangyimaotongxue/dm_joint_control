# dm_joint_control
该云台项目基于 ROS 2 Humble 和达妙 USB2CANFD 通信板实现。

### 通过 USB2CANFD 运行云台节点

`turret` 包仅保留 USB2CANFD 控制后端。USB2CANFD 使用 libusb，不访问
`/dev/ttyACM0`。

```text
src/turret/src/
└── moter_test/    # USB2CANFD 电机状态与动作测试节点
```

```bash
# 安装一次依赖
sudo apt install libusb-1.0-0-dev

source /opt/ros/humble/setup.bash
colcon build --packages-select turret
source install/setup.bash

# 查询 USB2CANFD 序列号
ros2 run turret dm_canfd_dev_sn
```

如果查询设备时提示权限不足，添加一次 udev 规则后重新插拔通信板：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-dm-usb2canfd.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

把查询到的 SN 填入
`src/turret/config/canfd_motor_params.yaml`，确认电机型号、CAN ID 和波特率后重新
构建，再运行：

```bash
PARAMS=$(ros2 pkg prefix turret)/share/turret/config/canfd_motor_params.yaml

# 默认仅检查反馈（command_enabled=false，不使能电机）
ros2 run turret dm_canfd_motor_run --ros-args --params-file "$PARAMS"

# 正负目标位置往复
ros2 run turret dm_canfd_motor_rerun --ros-args --params-file "$PARAMS"

# 将当前位置设为零点并保存（执行后自动退出）
ros2 run turret dm_canfd_set_zero_position --ros-args --params-file "$PARAMS"
```

只想先安全检查通信时，也可以不改参数文件，直接传入 SN。默认不会发送 MIT
控制指令：

```bash
ros2 run turret dm_canfd_motor_run --ros-args -p device_sn:=你的设备SN
```

确认电机型号、ID、零点和限位都正确后，再显式使能控制：

```bash
ros2 run turret dm_canfd_motor_run --ros-args \
  -p device_sn:=你的设备SN -p command_enabled:=true
```

USB2CANFD 的默认总线参数是仲裁域 1 Mbps、数据域 5 Mbps。电机自身的波特率也
必须由上位机设置为与之匹配的 CANFD 配置。仓库自带的 `libu2canfd.a` 是
x86-64 版本；在 Orange Pi/ARM 上必须换成厂商提供的 ARM 版本，并在构建时传入：

```bash
colcon build --packages-select turret --cmake-args \
  -DDM_U2CANFD_LIBRARY=/你的路径/libu2canfd.a
```

### 物料组成
||||
|---|---|---|
|达妙电机|DM-4310||
|达妙通信板|USB2CANFD||
|数据线|USB-A to USB-C|
|电机数据线|XT30（2 + 2）||
