# dm_joint_control
**该云台项目基于ros2-humble实现，但也包含了达妙电机原本的C++控制历程，不过云台的工作环境本身没有在C++的历程里面包含，你必须得使用ROS2的方式来操作云台**

### 通过ros2方式实现云台的基本操作
1. 使用git clone 下载源码
```bash
git clone https://github.com/wangyimaotongxue/dm_joint_control.git
```

2. 编译和构建ros2工作空间
```bash
colcon build
```

3. 使用ros2示例程序
```bash
# 添加环境变量
source ./install/setup.bash

# 给串口设置权限
sudo chmod -R 777 /dev/ttyACM0      # 也可以是dev/ttyACM*

# 启动这个测试节点
ros2 run turret dm_motor_run
```

### 使用达妙 C++ SDK测试电机
1. 构建达妙 C++ SDK
```bash
cd ./src/u2can/build/
cmake ..
make -j6
```

2. 电机测试程序
* 在原本的test_damiao.cpp之外我添加了如下部分程序，可以满足[设置关节零点、 检查关节、 关节转动测试]
```bash
src
    ├── dm_motor_rerun.cpp
    ├── dm_motor_run.cpp
    ├── dm_set_zero_position.cpp
    └── test_damiao.cpp
```

* 使用这些案例

**请在`/home/orangepi/dm_joint_control/src/u2can/build`下来操作** 
```bash

# 设置关节零点
./dm_set_zero_position

# 检查关节
./dm_motor_run

# 关节转动测试
./dm_motor_rerun

```
### 物料组成
||||
|---|---|---|
|达妙电机|DM-4310||
|达妙通信板|USB2CAN||
|数据线|USB-A to USB-C|
|电机数据线|XT30（2 + 2）||










