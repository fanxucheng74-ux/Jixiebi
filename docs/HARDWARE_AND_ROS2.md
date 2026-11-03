# 硬件连接与 ROS 2 入门

本页将实物连接、只读查询与运动分开。若尚未确认接线、总线供电和电平，请先对照设备厂商文档；不要依据通用示意图猜测插针或电压。

## 硬件连接原则

使用适配开发板的电源，并使用与舵机和机械臂控制底板额定值匹配的独立电源。将已确认兼容的 USB 总线转接板连接开发板的 USB Host，再按厂家线序接舵机总线。USB 供电、舵机供电和隔离是否存在取决于实际硬件；不要认为识别到 USB 转串口就已正确给舵机供电。

第一次接通后查看系统识别情况，串口名称以你的系统为准：

```bash
# 以下在 RK / Linux 终端执行；只枚举设备，不发送舵机控制帧。
lsusb
ls -l /dev/ttyUSB* /dev/serial/by-id/ 2>/dev/null
id
```

不同转接板可能显示不同名称；`/dev/ttyUSB0` 只是示例。Linux 上设备权限由实际所属组和系统规则决定，`dialout` 是否生效可用 `id` 核对。一个串口同时只允许一个项目硬件进程使用；建议锁不能阻止不遵守锁的其他程序。

## 实物只读查询

确认型号、线序、独立电源、机械臂状态、串口权限和项目依赖后，在项目根目录、对应 Python 环境下执行（替换端口）：

```bash
# 例如 Linux 上的 /dev/ttyUSB0；实际端口必须按现场替换。
python -m arm_project.cli.read_arm_state --port /dev/ttyUSB0
```

此命令会发送读取请求，不发送位置运动目标。成功时输出 `SOURCE: HARDWARE`、`STATE: OK` 和各轴状态；失败时查看电源、线序、串口占用、端口权限及日志，不自动重发运动。不要与 ROS 硬件节点同时运行。

## ROS 2 Jazzy 状态节点

下例需要 Ubuntu 24.04、ROS 2 Jazzy、colcon、系统 Python 能导入 `serial`，以及项目源码已复制到**当前设备**。在项目根目录执行：

```bash
# 首次构建：将当前目录记录为项目根。
export ARM_PROJECT_ROOT="$PWD"
source /opt/ros/jazzy/setup.bash
cd "$ARM_PROJECT_ROOT/ros2_ws"
colcon build --packages-select jixiebi_driver
source install/setup.bash

# 加入项目源码；保留 source ROS 后已有的 PYTHONPATH。
export PYTHONPATH="$ARM_PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

# 按实际设备替换 /dev/ttyUSB0；该节点只读并占用串口。
ros2 run jixiebi_driver raw_state --ros-args -p port:=/dev/ttyUSB0 -p period_s:=2.0
```

另开一个相同设备的终端，加载 ROS 环境后读取一次数据：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo --once --full-length /jixiebi/servo_states_raw
```

状态节点发布 `std_msgs/msg/String` 包装的 JSON：包含各轴原始 P、状态和主机接收时间；六轴是依次读取，不能当作同一时刻角度。按 Ctrl+C 退出节点并释放串口。节点进程存在不代表数据仍在更新；超时、停止发布和错误必须分别处理。

如果提示 `No module named rclpy`，先核对使用 ROS 匹配的系统 Python，并重新 `source /opt/ros/jazzy/setup.bash`；不要在 source 后用 `export PYTHONPATH=...` 覆盖 ROS 路径，应采用上面的追加方式。

## 运动控制边界

项目包含单次预审的 ROS 夹爪和连续单轴入口。真实运动不能直接套用别人的舵机 ID、P 范围、动作计划和 budget；需基于自己的设备和场地确认起点、完整路径、供电、边界及反馈，使用独立一次性计划，现场监管并保存原始记录。运动期间的取消/停止是软件请求，不等于已验证的物理急停；关机或断开网络也不能作为实际停止证明。

在公开检出和新设备上，先完成只读与路径审阅；本页故意不提供一键运动命令。ROS 触发服务没有内建身份认证，使用前需要控制网络访问。
