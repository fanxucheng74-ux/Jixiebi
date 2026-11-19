# 第十七课 · 用 FollowJointTrajectory 播放、查看进度和取消

你已经看过肘关节从 0° 弯到 −10°，再在第 4 秒回到 0°。这次保持模型、路点和插值都相同，把一次播放交给一个有身份的 ROS 2 Action 目标。**所有操作都在电脑 WSL / ROS 2 Humble；RK 可以关机。** 初始角沿用第 198 批标称姿态。[数据索引](ARM_FACTS.md)

## 在 WSL 写哪几个文件

从 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/` 开始读。

| 文件 | 先看什么 |
|---|---|
| [src/lesson17_follow_client.cpp](../../ros2_ws/src/jixiebi_learning/src/lesson17_follow_client.cpp) | 终端 B 的入口。`run()` 发 Goal，收 Feedback；保留句柄，可对这一次播放发起取消并取 Result。 |
| [src/lesson17_follow_server.cpp](../../ros2_ws/src/jixiebi_learning/src/lesson17_follow_server.cpp) | 终端 A 的播放器。`review()` 接纳目标，`render()` 发布一帧和一条反馈，`tick()` 决定继续、完成或取消。 |
| [src/trajectory_playback.hpp](../../ros2_ws/src/jixiebi_learning/src/trajectory_playback.hpp) | 第十五课的 `make_preview()`、第十六课的 `check_preview()` 和 `positions_at()`。同一份三次插值供两种播放器使用。 |
| [launch/lesson16_playback.launch.py](../../ros2_ws/src/jixiebi_learning/launch/lesson16_playback.launch.py) | 加 `input:=action` 就选第十七课服务端，原来的默认入口仍是 Topic。两个入口使用独立的话题和同一份第十二课 Xacro。 |

### 1 · 客户端把三个路点装入一个 Goal

打开客户端，在 `run()` 里找这两段：

```cpp
// Goal.trajectory 就是上一课熟悉的 JointTrajectory，时间从本次目标起点算起。
Follow::Goal goal;
goal.trajectory = lesson_trajectory::make_preview(-10.0);

// options 中已注册反馈回调；发送后等服务端接纳，取回此次目标的句柄。
auto admission = client->async_send_goal(goal, options);
```

`Follow` 是 `control_msgs::action::FollowJointTrajectory` 的类型别名。反馈含 `joint_names`、`desired`、`actual`、`error`。客户端从 `desired.time_from_start` 读取当前轨迹秒数并显示为 `t=已过/总长`。本例按自己发送的关节顺序读取第 3 列肘角，单位 rad。可以在 B 运行 `ros2 interface show control_msgs/action/FollowJointTrajectory` 对照本机定义，三段依次对应 Goal、Result、Feedback。[本机对应的官方接口定义](https://github.com/ros-controls/control_msgs/blob/4.9.0/control_msgs/action/FollowJointTrajectory.action)

### 2 · 服务端接受目标后，每 20 ms 做一次插值

构造函数给 `create_server<Follow>()` 注册三个回调，依次负责**审查请求、答复取消、开始被接纳的目标**。上一目标还在运行时，新请求会被拒绝。本课接受零起始时间戳、从 0 秒开始并严格递增的四轴位置路点；显式多自由度轨迹或自定义容差会被拒绝。

```cpp
// 接纳回调：保存这一次的句柄并记下单调时钟起点，马上发布第一帧。
active_ = handle;
start_ = Clock::now();
render(0.0);
```

之后计时器调用 `tick()`。每次用实际经过的秒数进入 `render()`。同一帧数据走两条路：

```cpp
// 路径一：从目标取轨迹，计算当前关节角，经 JointState → TF → RViz2 更新模型。
const auto &path = active_->get_goal()->trajectory;
frame_.name = path.joint_names;
frame_.position = lesson_trajectory::positions_at(path, seconds);
publish_frame();
```

```cpp
// 路径二：给持有该目标句柄的客户端返回当前进度，反馈与显示使用同一帧。
auto feedback = std::make_shared<Follow::Feedback>();
feedback->header = frame_.header;
feedback->joint_names = frame_.name;
feedback->desired.time_from_start = rclcpp::Duration::from_seconds(seconds);
feedback->desired.positions = frame_.position;
feedback->actual = feedback->desired;  // 理想模型直接应用该姿态。
feedback->error.time_from_start = feedback->desired.time_from_start;
feedback->error.positions.assign(frame_.name.size(), 0.0);
active_->publish_feedback(feedback);
```

A 的 `JointState` 发到 `/jixiebi/learning/follow/joint_states`，B 的反馈走 Action `/jixiebi/learning/follow_joint_trajectory`。打印出的 `model` 对应 `actual`，它记录模型采用的关节角，不提供编码器或 RViz 窗口的独立测量。底层约每 20 ms 更新一次，B 每约半秒打印一行，方便读数。

### 3 · 同一个句柄负责取消和取结果

B 在达到你设定的进度时调用 `client->async_cancel_goal(handle)`。A 答复受理后，Goal 进入 `CANCELING`。下一次 `tick()` **先查取消，再推进时间**。下面是决定模型能否停住的几行：

```cpp
if (active_->is_canceling()) {
    // 保留最后发布的 frame_，只结束这个目标。后续定时器仍发布相同角度。
    auto result = std::make_shared<Follow::Result>();
    result->error_string = "模型保持最后发布的姿态";
    active_->canceled(result);
    active_.reset();
    return;
}
```

自然完成时，A 先发布精确的末点，再调用 `active_->succeed(result)`。B 从 `async_get_result(handle)` 的返回值读取最终状态。看到 `CANCEL_ACCEPTED` 说明请求已受理，看到 `RESULT=CANCELED` 才说明任务已结束。FollowJointTrajectory 的业务错误码里没有专用的取消码，判断取消看 Action 终态。

### 4 · WSL 终端 A，编译并启动服务端与模型

当前电脑已经有本课依赖。换到新的 Humble 环境，如果提示 `control_msgs` 缺失，先在 **WSL** 执行 `sudo apt-get install ros-humble-control-msgs`。只在第一次或改完 C++ 后构建：

```bash
# WSL 终端 A，从项目的 ROS 工作区编译。
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to jixiebi_learning
```

**新开 WSL 终端 A、B，各执行一次以下环境命令。** A 如果已打开就用原窗口。

```bash
# 加载 Humble 和本项目的程序，沿用第十五、十六课的本机回环。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=22 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/config/wsl_loopback.xml
```

```bash
# 只在 WSL 终端 A 执行。保持此窗口，等 RViz2 显示接近直立的彩色整臂。
ros2 launch jixiebi_learning lesson16_playback.launch.py input:=action
```

A 应打印“模型 Action 就绪，等待轨迹目标”。启动文件仍叫 lesson16，是为了复用你已经看到的 RViz 配置；`input:=action` 会启动第十七课服务端。B 若提示找不到服务端，先看 A 的这行和两个窗口的域及回环设置。

### 5 · WSL 终端 B，先看完整往返，再试中途取消

```bash
# 第一次：只发一个目标，看到结果后自动回到命令行。
ros2 run jixiebi_learning lesson17_follow_client
```

看 B 的输出顺序：`GOAL_ACCEPTED` → 多行 `t=... desired=... model=...` → `RESULT=SUCCEEDED error_code=0`。模型在约 2 s 弯到 −10°，在 4 s 回到 0°。

```bash
# 第二次：在反馈时间达到 1 秒时，请求取消当前目标。A 继续运行。
ros2 run jixiebi_learning lesson17_follow_client --ros-args -p cancel_after_s:=1.0
```

B 应先后出现 `CANCEL_REQUESTED`、`CANCEL_ACCEPTED`、`RESULT=CANCELED`。RViz2 中肘角约在 −5° 附近，等到原定的第 4 秒以后也应保持。定时调度和消息传递可能使最后一帧比理论的 1.000 秒稍晚。之后如果再发新目标，模型从**新目标显式提供的第一个路点**开始本次播放。

可在 B 读一次保持下来的角度：

```bash
# B 已返回提示符。四个数的第三列是 elbow_flex，单位 rad。
timeout 5s ros2 topic echo /jixiebi/learning/follow/joint_states sensor_msgs/msg/JointState --once --field position
```

客户端已退出时，服务端仍在 A；看完在 A 按 Ctrl+C 关闭。本课的验收范围是电脑模型中的 Action 通信、插值和取消保持。

**留一道题**　若把 `cancel_after_s` 改成 `3.0`，理想肘角也约为 −5°。这次取消前肘是正在弯曲还是正在伸直？沿同一条轨迹找出 1 s 与 3 s 各自位于哪两个路点之间。

接下来读[第十八课 · 从取消后停住的关节角接着走](CPP_LESSON18_FROM_STATE.md)，在新客户端里把刚收到的模型状态写成首点。
