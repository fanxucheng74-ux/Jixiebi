# 第二十一课 · 让 RViz2 的黄色球心沿直线走

上一课你输入 −5° 那组 XYZ 和工具俯仰，肘回到了约 −5°。这次要验证沿途：在电脑 WSL 中，让球心从左端 (−70, 180, 445) mm 走到右端 (+70, 180, 445) mm。两个点都在 `base_link` 中，Y、Z 保持不变。沿用[事实表](ARM_FACTS.md)里的第十二课模型和第二十课的逆解。工具俯仰固定为 55°，相对于随底座旋转的肩肘平面；底座转动时工具在世界中的朝向也会变化。

## 第一步 · 用两端点写出中间点

在电脑 WSL 打开 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/lesson21_cartesian_line.cpp`，从 `make_line()` 开始。先写这两行，再看循环其余部分。[带中文注释的主程序](../../ros2_ws/src/jixiebi_learning/src/lesson21_cartesian_line.cpp)

```cpp
// i 从 0 到 segments。先转为 double，否则整数除法会丢掉中间的小数。
const double u = static_cast<double>(i) / segments;
const auto xyz = left + (right - left) * u;  // KDL 使用米；只改变 X。
```

分 12 段需要 13 个路点。亲手代入 `i=6`：此时 `u=0.5`，球心必须经过 (0, 180, 445) mm。若只给左右端点，播放器并没有收到这个中点条件。

## 第二步 · 在每个路点调用上一课逆解

```cpp
const auto q = lesson_ik::solve(geometry, xyz, pitch, previous);
lesson_ik::check_solution(chain, q, xyz, pitch);  // 用第十九课的正解检查球心和工具方向。
```

`previous` 第一次来自新收到的模型关节状态。求完一个路点后，把该点四个角写回 `previous`，让下一个点选靠近上一点的肩肘姿态。点和时间全部检查通过，才发出**一个** `FollowJointTrajectory` 目标。目标包含从当前姿态到左端的进场段，以及从左到右的横移段。再次实验时，程序会自己回到左端。[两课共用的几何逆解](../../ros2_ws/src/jixiebi_learning/src/lesson_ik.hpp)

横移通常用 4 秒，时间按段分配，必要时延长以满足课程模型的速度限制。播放器对关节角逐段做三次插值，每个路点处的插值速度会降到零。分段增加可改善路径形状；匀速或路点间的连续速度是另一项控制要求。

## 第三步 · 用反馈画出球心走过的路

进场完成后，`options.feedback_callback` 读取本次 Action 的 `actual.positions`。这些是本机播放器这一帧用于显示的角度。将四轴角填入 KDL，再用 `fk.JntToCart(q, tip)` 和 `view.observe(tip.p)` 把实际显示的球心位置加入橙线。图中两种颜色分别来自不同的数据。[两课共用的反馈回调](../../ros2_ws/src/jixiebi_learning/src/lesson_line_run.hpp) · [可视化代码](../../ros2_ws/src/jixiebi_learning/src/lesson21_path_view.hpp)

| RViz2 显示或终端字段 | 读法 |
|---|---|
| 青蓝点 | 理想直线上的 21 个等距点，两次实验相同；它们是视觉标尺。 |
| 橙线 | 按收到的横移反馈帧做正运动学后连成的线。进场段不画入。 |
| `MAX_LINE_ERROR_MM` | 每个反馈点到有限目标线段的最短距离，取已采样帧的最大值，单位 mm。 |
| `END_ERROR_MM` | 播放结束后另读一帧 JointState，计算球心到右端点的距离。 |

标记类型可对照 Humble 消息中的 `SPHERE_LIST` 和 `LINE_STRIP`。[ROS 2 Marker 字段定义](https://github.com/ros2/common_interfaces/blob/humble/visualization_msgs/msg/Marker.msg)

## 第四步 · 两个 WSL 终端实际运行

**编译**：首次克隆或修改 C++ 后，在 WSL 终端 B 执行：

```bash
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to jixiebi_learning
```

如果上一课的模型仍在 A 运行，先在 A 按 Ctrl+C。本课 A 会增加轨迹显示。**两个 WSL 终端各执行一次**下面的环境设置：

```bash
# 加载 ROS 2 Humble、课程程序，并使用第十九、二十课的本机域。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=41 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**终端 A** 启动一份模型，并保持运行。RViz2 的 Displays 应出现 `Lesson21 Path`。

```bash
ros2 launch jixiebi_learning lesson16_playback.launch.py input:=cartesian_line
```

**终端 B** 在同一窗口依次做两次对照。第一条结束后先看一眼橙线，再执行第二条。

```bash
# 对照组：横移只给左右两个端点，观察橙线在中间怎样偏离蓝点。
ros2 run jixiebi_learning lesson21_cartesian_line --ros-args -p segments:=1

# 分段组：自动回到左端，换成 12 段；旧橙线会被这次的反馈替换。
ros2 run jixiebi_learning lesson21_cartesian_line --ros-args -p segments:=12

# 可选：再用第十九课独立回读球心，预期 XYZ_MM=[70.000, 180.000, 445.000]。
ros2 run jixiebi_learning lesson19_tool_position
```

两端姿态关于 X=0 对称。只分一段时，肩、肘、腕可以保持不变，底座的转动会把球心带出一段圆弧。水平半径为 `hypot(70,180) ≈ 193.13 mm`，圆弧中部比要求的 180 mm 多出约 13.13 mm。12 段在这次实验的中点也要求逆解，肩、肘和腕会随底座协同调整；路点间仍有小的插值误差。数字表示 WSL 模型的几何偏差，实机精度需另测。

本机 WSL 对照中，两组各收到 201 个横移反馈点。1 段的最大采样偏差为 **13.132 mm**，12 段为 **0.098 mm**；两组终点误差均显示 **0.000 mm**。反馈时刻会随调度略有差别。黄色球可能挡住线的端部，观察中段最容易判断。

**自己改一次**：把 `segments` 改成 4。先预测哪一个指标会明显变化，再看 B 的数字。如果橙线和蓝点重叠不容易分辨，拖动视角，从上方看 Y 方向的偏移；也可保留当前窗口，重跑 1 段作对照。
