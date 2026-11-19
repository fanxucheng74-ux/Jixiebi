# 第十五课 · 用 C++ 写第一条关节轨迹

第十四课回答“机械臂模型现在是什么姿态”。这节课回答“模型应该在第几秒到达什么姿态”。我们把第 198 批的旧姿态作为例题起点，构造一条四关节、三个时间点的 `trajectory_msgs/msg/JointTrajectory`，在另一个终端读出。所有操作都在**电脑 WSL Ubuntu 22.04 / ROS 2 Humble**，RK 可以关着。课程话题只供查看，RViz2 的第十四课窗口仍显示它原来订阅的反馈。[已有尺寸与实机记录](ARM_FACTS.md)

## 第一步 · 先在纸上写三行

四列依次是底座、肩、肘、腕。下面用度方便阅读；写入消息时用弧度。0 秒这一行取自实机反馈经旧模型换算的标称角；2 秒和 4 秒是我们给预览设置的目标。三个点同时随一条消息发送。

| 从起点起算 | base_yaw | shoulder_pitch | elbow_flex | wrist_pitch |
|---|---:|---:|---:|---:|
| 0 s | −0.135° | 89.73° | 0° | +0.27° |
| 2 s | −0.135° | 89.73° | −20° | +0.27° |
| 4 s | −0.135° | 89.73° | 0° | +0.27° |

## 第二步 · 在 WSL 写点和列名

在电脑 WSL 打开 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/trajectory_playback.hpp`，对照下面三段写 `make_preview()`。第十五课的 `main()` 调用它构造消息，第十七课也直接调用同一个函数。[轨迹构造](../../ros2_ws/src/jixiebi_learning/src/trajectory_playback.hpp) · [第十五课 main 与发布器](../../ros2_ws/src/jixiebi_learning/src/lesson15_trajectory_preview.cpp)

```cpp
Trajectory trajectory;  // 一条轨迹共用下面这份列名
trajectory.joint_names = {"base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch"};
Point start;            // 一个时间点要包含四列位置
start.positions = {-0.135 * kRadPerDegree, 89.73 * kRadPerDegree, 0.0, 0.27 * kRadPerDegree};
start.time_from_start.sec = 0;
```

`Trajectory`、`Point` 是文件开头给两个标准消息类型取的短名。每一列都按 `joint_names` 中相同下标解释。下标从 0 开始，这份名单里肘是第 3 列，所以下标是 2。这里先前已经完成 P→角度换算，乘 `π / 180` 只负责度→弧度。[Humble 轨迹定义](https://github.com/ros2/common_interfaces/blob/humble/trajectory_msgs/msg/JointTrajectory.msg) · [轨迹点定义](https://github.com/ros2/common_interfaces/blob/humble/trajectory_msgs/msg/JointTrajectoryPoint.msg)

```cpp
Point bend = start;  // 复制四个值，随后只改肘。start 仍保留原值。
bend.positions[kElbow] += elbow_offset_deg * kRadPerDegree;
bend.time_from_start.sec = 2;
Point back = start;  // 返回原位时，角度相同，到达时间不同
back.time_from_start.sec = 4;
trajectory.points = {start, bend, back};
return trajectory;
```

`time_from_start` 从整条轨迹的起点计时。因此回程耗时 `4 − 2 = 2 s`。ROS 消息描述路点，点与点之间如何插值、实际电机是否跟得上，是后续执行器要解决的问题。本例只填目标位置，速度、加速度、力矩数组均留空。

## 第三步 · 在 main 里发出去

```cpp
// 保留最近一条消息。发布节点还在运行时，后启动的订阅者也可以读到它。
const auto qos = rclcpp::QoS(1).reliable().transient_local();
auto publisher = node->create_publisher<Trajectory>("/jixiebi/learning/trajectory_preview", qos);
publisher->publish(make_preview(offset));  // 先构造一整条轨迹，然后只发布一次
rclcpp::spin(node);                        // 保持节点和缓存存活
```

`offset` 来自启动参数 `elbow_offset_deg`，默认 −20.0°。起始肘角为 0°，示例用 URDF 的 ±90° 检查这个目标值。协议 P、串口和实物行程都不参加本例。构建上，`package.xml` 声明了 `trajectory_msgs`，`CMakeLists.txt` 的 `add_executable` 将源文件注册为 `lesson15_trajectory_preview`，并为该目标链接 ROS 消息依赖。[构建文件](../../ros2_ws/src/jixiebi_learning/CMakeLists.txt) · [包依赖](../../ros2_ws/src/jixiebi_learning/package.xml)

## 第四步 · WSL 终端 A 编译，启动后留着窗口

```bash
# 在电脑 WSL 中进入已有工作区。修改 C++ 后重新编译，再加载安装结果。
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to jixiebi_learning && source install/setup.bash

# 两个 WSL 终端都用域 22；本机回环配置把 DDS 通信限定在 127.0.0.1。
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=22 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/config/wsl_loopback.xml
ros2 run jixiebi_learning lesson15_trajectory_preview
```

预计 A 显示“轨迹预览已发布：3 个点，0 / 2 / 4 秒，肘关节偏转 -20.00°”。如果已经编译过这版源码，加载 `install/setup.bash` 后可直接运行。

## 第五步 · WSL 终端 B 读消息，自己改一个值

```bash
# 新开电脑 WSL 终端；此处无需 SSH，也无需启动 RK 发布器。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=22 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/config/wsl_loopback.xml

# 显式读取发布器保存的那一条；打印完会自动退出。
timeout 15s ros2 topic echo /jixiebi/learning/trajectory_preview trajectory_msgs/msg/JointTrajectory \
  --once --full-length --qos-durability transient_local --qos-reliability reliable --qos-depth 1
```

在 B 中找三个 `positions`，读每组第 3 个数，应依次为 **0、约 −0.349066、0 rad**。若超时，先确认 A 仍在前台，再核对两个终端的域号、回环 XML 路径和 B 的 `transient_local`。本机镜像网络上默认配置曾超时，按上面显式指定回环后已收到完整轨迹。下次回到第十四课看实物时，两端按第十四课恢复域 21 和热点配置。

现在自己改一次：在 A 按 Ctrl+C 后执行下面这行，在 B 再执行一次刚才的 `echo`。只改启动参数无需编译。

```bash
# WSL 终端 A：沿用本窗口环境，让预览肘角只偏转 10°。
ros2 run jixiebi_learning lesson15_trajectory_preview --ros-args -p elbow_offset_deg:=-10.0
```

验收时只对照两件事：中间点肘角应变为约 −0.174533 rad；最后一个点仍在第 4 秒返回起点。结束后在 A 按 Ctrl+C。

接着用同一条消息驱动模型，见[第十六课 · 按时间插值播放](CPP_LESSON16_TRAJECTORY_PLAYBACK.md)。
