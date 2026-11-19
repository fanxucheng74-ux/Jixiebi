# 第十八课 · 从取消后停住的关节角接着走

你在第十七课看到，同样约 −5° 的位置可以有两个运动方向。按模型角度定义，第 1 秒从 0° 去 −10°，角速度为负；第 3 秒从 −10° 返回 0°，角速度为正。取消后，模型停在最后显示的角度。这一课写一个规划两点轨迹的 C++ 客户端，让下一次弯肘从这个角度开始。

**位置：电脑 WSL，Ubuntu 22.04 / ROS 2 Humble。** A 继续使用上课的模型、插值器和 Action；B 读取模型的 JointState。RK 可以关机。沿用第十二课 URDF 和已有数据。[机械臂数据索引](ARM_FACTS.md)

## 1 · 在客户端读一帧

文件在 WSL 的 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/lesson18_from_state.cpp`。先看 `run()`。客户端等到 Action 服务端就绪，再从它的模型状态话题取一帧。[完整客户端](../../ros2_ws/src/jixiebi_learning/src/lesson18_from_state.cpp)

```cpp
State state;  // 这是 sensor_msgs::msg::JointState，name 和 position 按下标对应。
if (!rclcpp::wait_for_message(state, node, "/jixiebi/learning/follow/joint_states", 3s)) {
    throw std::runtime_error("3 秒内没有收到模型状态");
}
```

`3s` 是等消息的最长时间；下一行从 ROS 当前时间减去消息时间戳，检查收到的这一帧有多旧。本课两端在同一台 WSL，接受最近 0.5 秒且没有超前的模型帧。这个时间窗口用于本机练习。[Humble 的 wait_for_message 接口](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/wait_for_message.hpp) · [JointState 字段](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg)

## 2 · 亲手写两点轨迹

同一文件中找到 `plan_from_state()`。最值得自己写的是下面这几行。示例中 `elbow` 由 `std::find` 按名字找到，前面已检查关节存在、位置数组和名称数组长度相同。

```cpp
path.joint_names = state.name;
lesson_trajectory::Point start;
start.positions = state.position;  // 第 0 秒：复制当前的四个角，单位 rad。
auto finish = start;               // 先复制，这样另外三轴的目标就等于各自的起点。
const auto column = std::distance(state.name.begin(), elbow);
finish.positions[column] = target_deg * lesson_trajectory::kRadPerDegree;
finish.time_from_start.sec = 2;
path.points = {start, finish};
```

比如停在 −5.1° 时，参数 `elbow_target_deg:=-10.0` 给的是**绝对角度**。新轨迹只需要在 2 秒内再走约 −4.9°。原三次曲线从静止起步，到目标时速度回到零；在第 1 秒，新轨迹大约走到 −7.55°。若已精确处于指定模型角，客户端打印 `ALREADY_AT_TARGET` 并退出。

## 3 · 理解服务端为什么还要比较一次

在客户端读完模型和服务端收到目标之间，姿态可能改变。启动时选 `input:=checked_action`，服务端就会在 `review()` 接纳目标之前调用 `start_mismatch()`，按关节名逐轴比较“轨迹第一个点”和“此刻保持的模型帧”。有任何一轴差值超过 `1e-6 rad` 就拒绝该目标，继续显示原姿态。这个量只作同一软件模型的浮点比较，实际舵机容差需结合实测。[服务端起点核对](../../ros2_ws/src/jixiebi_learning/src/lesson17_follow_server.cpp)

第十七课的固定目标把 0° 写作第一个肘角；第十八课客户端则把刚采到的姿态写进首点。两者使用相同的 `FollowJointTrajectory` 类型。每个新目标都从自己的第 0 秒开始计时。

## 4 · WSL 终端 A 打开模型

若上一课 A 的 launch 还在运行，先在那个窗口按 Ctrl+C。两课复用同一个 Action 名称，只运行一份服务端。本课需要在新启动的服务端上看到“起点核对已开启”。

首次或修改 C++ 后，在 A 编译。本机交付时已构建。

```bash
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to jixiebi_learning
```

两个新 WSL 终端 **A、B 各执行一次**：

```bash
# 加载工作区，复用课程用的域 22 和本机回环。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=22 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/config/wsl_loopback.xml
```

**只在 A 执行**，等彩色模型打开：

```bash
# 在接纳每个目标时比较当前四轴与首个路点；保持窗口运行。
ros2 launch jixiebi_learning lesson16_playback.launch.py input:=checked_action
```

## 5 · WSL 终端 B 连做两次，观察接续

```bash
# ① 从初始姿态运行旧练习，在 1 秒时取消，等 B 出现 RESULT=CANCELED。
ros2 run jixiebi_learning lesson17_follow_client --ros-args -p cancel_after_s:=1.0

# ② 使用新客户端，从刚停住的位置继续弯到 −10°。
ros2 run jixiebi_learning lesson18_from_state --ros-args -p elbow_target_deg:=-10.0
```

B 会打印 `START_DEG=...`、`TARGET_DEG=-10.000`，约两秒后出现 `RESULT=SUCCEEDED`。起点应接近 −5°，随实际收到取消的时刻略有变化。看 A 的绿色前臂，应直接从停留处继续弯，不先弹到 0°。也可以把紧接取消后的第二条命令的目标改为 0°，比较方向。

```bash
# 可选练习：此刻已经在 −10°，用同一个客户端平滑回到 0°。
ros2 run jixiebi_learning lesson18_from_state --ros-args -p elbow_target_deg:=0.0
```

如果在**非零肘角**下发送旧的第十七课固定起点轨迹，B 会提示 `GOAL_REJECTED`，A 应指明 `elbow_flex` 起点不符；用第十八课客户端可重新读取当前位置。看完在 A 按 Ctrl+C。所有状态和结果都来自 WSL 模型。

接着沿用这个模型，用[第十九课的正运动学程序](CPP_LESSON19_TOOL_POSITION.md)比较弯肘前后工具点的 XYZ。
