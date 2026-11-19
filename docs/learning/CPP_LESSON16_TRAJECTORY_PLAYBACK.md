# 第十六课 · 用 C++ 按时间播放关节轨迹

**本课目标**　在电脑 WSL 的 RViz2 里，看绿色前臂和腕部相对蓝色上臂缓慢弯曲，在第 2 秒换向，第 4 秒返回。两个终端都运行 Ubuntu 22.04 / ROS 2 Humble，RK 可以关机。

先读过[机械臂数据索引](ARM_FACTS.md)。起始肩角沿用第 198 批的标称 89.730°，所以一开始应该看到手臂接近直立。预览采用[第十五课原发布器](../../ros2_ws/src/jixiebi_learning/src/lesson15_trajectory_preview.cpp)和[第十二课彩色模型](../../ros2_ws/src/jixiebi_learning/urdf/lesson12_arm.urdf.xacro)。

## 1 · 先看输入和输出

| 谁在运行 | 接收什么 | 产生什么 |
|---|---|---|
| 终端 B 的第十五课节点 | 启动参数 `elbow_offset_deg:=-10.0` | 一条 `JointTrajectory`，包含 0、2、4 秒的四轴位置 |
| 终端 A 的新 C++ 播放器 | `/jixiebi/learning/trajectory_preview` | 约每 20 ms 向 `/jixiebi/learning/playback/joint_states` 发一条 `JointState` |
| 终端 A 的 `robot_state_publisher` | 四个关节的模型角和 URDF | 连杆之间的 TF |
| 终端 A 的 RViz2 | 模型描述、TF | 每根连杆随时间变化的姿态 |

这一课练的是**时间插值**。发送者给定路点和到达时刻；播放器回答“现在过去了 1.4 秒，这四个关节应该显示多少弧度？”整个过程只使用电脑的本机回环。

想核对字段时，直接看 Humble 接口。路点的 `time_from_start` 指定累计到达时间，输出消息用同一下标关联关节名和位置。[JointTrajectoryPoint 定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/trajectory_msgs/msg/JointTrajectoryPoint.msg) · [JointState 定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/JointState.msg)

## 2 · WSL 写订阅回调，保存轨迹并记下起点

打开 [lesson16_trajectory_player.cpp](../../ros2_ws/src/jixiebi_learning/src/lesson16_trajectory_player.cpp)。从 `main → TrajectoryPlayer() → receive()` 开始读。在 WSL 工作区中，文件位于 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/lesson16_trajectory_player.cpp`。

```cpp
// 构造函数里注册订阅。第十五课发一次并保留末条，这里用相同的 QoS。
preview_ = create_subscription<Trajectory>(
    "/jixiebi/learning/trajectory_preview", rclcpp::QoS(1).reliable().transient_local(),
    [this](Trajectory::ConstSharedPtr msg) { receive(msg); });
```

`main()` 里的 `rclcpp::spin()` 等消息和计时器事件。消息来了才调用 `receive()`。它先用 `check_preview()` 检查关节名、角度、列数与到达时间，然后执行下面几行。

```cpp
// 保存整条消息；这里取一次起播时间，发布首帧让模型就位。
path_ = msg;
start_ = Clock::now();
playing_ = true;
publish_positions(path_->points.front().positions);
```

第十五课用零时间戳表示收到即可开始。本课约定输入只含四轴位置，路点时间从 0 开始并严格递增。播放中到来的下一条预览会被忽略，当前模型可以连续走完这一遍。

## 3 · WSL 写 positions_at()，先算在哪一段，再算这一帧的位置

打开同目录的 [trajectory_playback.hpp](../../ros2_ws/src/jixiebi_learning/src/trajectory_playback.hpp)。`positions_at()` 和 `check_preview()` 由第十六课 Topic 播放器和第十七课 Action 服务端共用。

`a`、`b` 是包围当前时刻的两个路点。第 3 秒属于返回段，此时 `a` 在第 2 秒、`b` 在第 4 秒。用“已走过的本段时间”除以“本段总时间”，得到无单位的进度 `u = (3−2)/(4−2) = 0.5`。

```cpp
// u 是本段时间进度，s 是经过缓入缓出处理的位置进度。
const double u = (elapsed - seconds_at(a)) / (seconds_at(b) - seconds_at(a));
const double s = u * u * (3.0 - 2.0 * u);
auto positions = a.positions;
for (std::size_t j = 0; j < positions.size(); ++j) {
    // 对同一列做插值，沿用输入的关节顺序。没有角差的关节自然保持原位。
    positions[j] += s * (b.positions[j] - a.positions[j]);
}
```

三次曲线的斜率为 `6u(1−u)`，在段首、段尾都为零。这正好适合肘部到达最弯处、减速换向再返回。直接用线性比例 `s=u` 时，中间点前后会从匀速弯曲瞬间切换到匀速伸直。这里让每个路点的速度都为零，适用于本课的往返动作。

| 自起播经过 | 模型肘角 | 可以观察到什么 |
|---|---:|---|
| 0 s | 0° | 第 198 批对应的起始姿态 |
| 0.5 s | −1.5625° | 刚起步，走过首段四分之一时间 |
| 1 s | −5° | 正在弯肘 |
| 2 s | −10° | 速度降为零，开始换向 |
| 3 s | −5° | 已经在伸直 |
| 4 s 及以后 | 0° | 保持最后姿态 |

## 4 · WSL 写 tick()，让时间驱动模型

```cpp
// 构造函数中建立 50 Hz 定时器；实际回调时刻允许有少量调度抖动。
timer_ = create_wall_timer(20ms, [this]() { tick(); });
```

每次进入 `tick()`，用单调时钟算出实际经过的秒数 `elapsed`，调用 `positions_at(*path_, elapsed)`。超过末点就直接取 `points.back().positions`。即使某帧迟到，也按实际时间取姿态。播放器继续发布末帧，便于 RViz2 持续显示。

```cpp
// publish_positions()：name 和 position 的同一列属于同一个关节。
JointState state;
state.header.stamp = now();  // 用 ROS 当前时间给这一帧标记时间，供 TF 使用。
state.name = path_->joint_names;
state.position = positions;  // 单位 rad。
states_->publish(state);
```

[启动文件](../../ros2_ws/src/jixiebi_learning/launch/lesson16_playback.launch.py)将状态发布器的 `/joint_states` 接到播放话题，将它和 RViz2 的 TF 接到同一组本课话题。它把第十二课 Xacro 展开一次，状态发布器用这份模型计算 TF，RViz2 从临时 URDF 文件读取同一份外形。[CMakeLists.txt](../../ros2_ws/src/jixiebi_learning/CMakeLists.txt)把新源码编译成 `lesson16_trajectory_player` 并安装。[package.xml](../../ros2_ws/src/jixiebi_learning/package.xml)声明所用依赖。

## 5 · WSL 终端 A，编译并打开模型

如果上一课的轨迹发布器仍在运行，先在它的窗口按 Ctrl+C。这样从 RViz2 窗口打开后才开始这一遍。

```bash
# 首次或修改 C++ 后编译；已经构建好时，只需加载两个 setup 文件。
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to jixiebi_learning && source install/setup.bash

# 复用第十五课的本机 Fast DDS 配置。两个终端须一致。
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=22 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/config/wsl_loopback.xml

# 启动播放器、状态发布器和 RViz2；等待窗口打开，再操作终端 B。
ros2 launch jixiebi_learning lesson16_playback.launch.py
```

终端 A 应显示“等待第十五课轨迹”。收到首帧前，活动连杆尚无 TF；窗口里可能只看到网格或部分连杆。

## 6 · WSL 终端 B，发送你刚验证的 −10° 轨迹

```bash
# 新开电脑 WSL 终端 B，加载已构建的程序，无需再次编译。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=22 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/config/wsl_loopback.xml

# 只发一条含三个路点的标准消息。发布器保持运行，模型在四秒内播放一次。
ros2 run jixiebi_learning lesson15_trajectory_preview --ros-args -p elbow_offset_deg:=-10.0
```

看绿色前臂相对蓝色上臂的夹角。A 应先显示“开始播放 3 个路点，全长 4.000 秒”，随后显示“播放完成，保持最后姿态”。想再看一次，保留 A，在 B 按 Ctrl+C 并重新执行最后一行即可。看完分别在 B、A 按 Ctrl+C。

若 A 一直显示等待，先核对 B 是否打印“轨迹预览已发布”以及两个窗口的域号和回环 XML。若 A 显示播放完成才看到窗口，按上面的方式重播一次。

**给自己的一道题**　在源码中找到 `u=0.25` 对应的那一帧，解释为什么 0.5 秒只弯了 1.5625°。能把“定时器的频率”“每个路点的到达时间”“这一帧的角度”分开说清楚，就掌握了这一课。

下一课给同一个插值函数接上 Action，练习[发送轨迹、看反馈和中途取消](CPP_LESSON17_FOLLOW_ACTION.md)。
