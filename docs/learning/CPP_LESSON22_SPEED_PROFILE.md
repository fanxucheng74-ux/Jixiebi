# 第二十二课 · 让末端在直线中段保持巡航速度

仍在电脑 WSL 的 Humble 模型上做实验。沿用[本机事实表](ARM_FACTS.md)里的 140 mm 横移：从 (−70, 180, 445) 到 (+70, 180, 445) mm，工具俯仰 55°，坐标系为 `base_link`。这次要回答的问题是：球心经过中间路点时，速度究竟有没有掉下来？

## 1 · 先解释分成 4 段时看到的快慢

[第 21 课](CPP_LESSON21_CARTESIAN_LINE.md)的每段使用 `3u²−2u³`。导数是 `6u(1−u)`，段首、段尾都为零。因此分得更细可以让线更直，也会产生更多次减速和起步。横移默认给各段分配 `4 s / 段数`，加起来仍是 4 s；当模型的关节限速要求更长时间时，个别段才延长。仅凭段数推不出瞬时速度或总用时。

## 2 · 先写“一秒走到哪里”

在 WSL 项目目录 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/` 打开 **lesson22_speed_profile.hpp**。先写 `Profile::distance_mm()`。这里的 `t` 从球心到达左端才开始计，`s(t)` 是沿线已经走过的毫米数。[带中文注释的时间律和速度测量](../../ros2_ws/src/jixiebi_learning/src/lesson22_speed_profile.hpp)

| 阶段 | 默认 35 mm/s 时 | 怎样算累计距离 |
|---|---|---|
| 加速 | 0～0.5 s | 从静止起步，`s = a·t²/2`，其中 `a = 35/0.5 = 70 mm/s²` |
| 巡航 | 0.5～4.0 s | 每秒走 35 mm，`s = V·(t−0.25)` |
| 减速 | 4.0～4.5 s | 用剩余时间算刹车距离，`s = 140−a·(T−t)²/2` |

速度曲线下面的面积等于距离。加速与减速各占半秒，平均速度各为 V/2，所以横移总时长为 `T = 140/V + 0.5`。举例：0.5 s 时已走 8.75 mm；1.5 s 时已走 43.75 mm。全程的平均速度为 140/4.5 ≈ 31.11 mm/s，只有中段巡航为 35 mm/s。[Modern Robotics 的梯形时间律示意](https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/)

## 3 · 把时间变成 XYZ，再变成关节角

从 [C++ 主程序](../../ros2_ws/src/jixiebi_learning/src/lesson22_timed_line.cpp) 进入两课共用的 [lesson_timed_line.hpp](../../ros2_ws/src/jixiebi_learning/src/lesson_timed_line.hpp)，找到 `make_timed_line()` 的 `line_sample`。下面四行把新知识接到你已经会的逆解上。

```cpp
const double u = profile.distance_mm(t) / profile.length_mm; // 此时已走过全长的比例。
const auto xyz = left + (right - left) * u;                  // 在同一条直线上取点，KDL 使用 m。
const auto q = lesson_ik::solve(geometry, xyz, pitch, seed); // 参考上一点的关节角选解。
lesson_ik::check_solution(chain, q, xyz, pitch);              // 用正解复核位置与工具方向。
```

每隔至多 50 ms 生成一个样本，再写入角度和 `entry_s + t`。所有路点随一次 Action 提交。进场从当前模型角平滑到左端。横移中的加速、巡航和刹车由全局的 `s(t)` 决定。新模式在相邻样本之间线性连接关节角，因此经过内部路点时不会重复套用零速起停曲线。有限采样会给反馈速度带来小幅波动，尤其在加减速段；这是一条离散近似的模型轨迹。

## 4 · 从反馈求速度

返回 **lesson22_speed_profile.hpp** 的 `SpeedMeter::sample()`。收到同一目标的相邻两帧，用正运动学得到的两个球心位置和反馈中的 `actual.time_from_start` 计算：

```cpp
const double dt = t - previous_t_;                   // 两帧实际相隔的秒数。
const auto d = tip - previous_tip_;                  // 位移向量，单位 m。
const double ds_mm = 1000.0 * std::hypot(d.x(), d.y(), d.z());
const double speed = ds_mm / dt;                     // 此帧间隔的平均速率，单位 mm/s。
```

第一帧仅保存。差分时采用真实时间差，巡航统计只收完全处于巡航区间的帧对。终端的 `REF_MM_S` 为同一时间间隔的理论距离差除以 dt，`MODEL_MM_S` 为 FK 反馈实算值。WSL 模型反馈约 20 ms 一帧，终端只节选打印，统计会使用全部有效帧。

## 5 · 两个 WSL 终端运行

首次克隆或修改 C++ 后，在 **WSL 终端 B** 编译一次：

```bash
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to jixiebi_learning
```

若第 21 课的 A 仍在运行，先在 A 按 Ctrl+C。然后在 **A、B 各执行一次**：

```bash
# 加载 Humble 与工作区；两端使用课程的 WSL 本机域。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=41 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**A** 启动支持时间样本的播放器和 RViz2。左栏勾选 `Lesson22 Path`，保持运行：

```bash
ros2 launch jixiebi_learning lesson16_playback.launch.py input:=speed_profile
```

**B** 只改一个变量做对照。先观察第一条的三阶段读数与最终橙线，再运行第二条：

```bash
# 35 mm/s 巡航；程序自动进场，从左端计时，横移预计 4.5 s。
ros2 run jixiebi_learning lesson22_timed_line --ros-args -p cruise_mm_s:=35.0

# 同一条线，巡航降到 25 mm/s；再次自动进场，横移预计 6.1 s。
ros2 run jixiebi_learning lesson22_timed_line --ros-args -p cruise_mm_s:=25.0

# 可选：用第十九课从新的 JointState 回读黄色球心，应为 (70, 180, 445) mm。
ros2 run jixiebi_learning lesson19_tool_position
```

读结果按三个问题来。**走得匀吗？** 看 `CRUISE_MIN_MM_S / CRUISE_MEAN_MM_S / CRUISE_MAX_MM_S` 是否都接近目标，以及加减速行是否先升后降。**走了多久？** 比较 `PLANNED_LINE_S` 和 `OBSERVED_LINE_S`；后者从第一帧横移反馈算到最后一帧，可能略短，进场时间另列。**走得直吗？** 看橙线是否贴着青蓝点，并核对 `MAX_LINE_ERROR_MM` 和 `END_ERROR_MM`，单位都是毫米。

## 本机 WSL 反馈对照

| 目标巡航 | 巡航反馈最小 / 平均 / 最大（mm/s） | 规划横移 / 反馈覆盖（s） | 最大采样线偏差 / 终点误差（mm） |
|---|---|---|---|
| 35 mm/s | 34.977 / 35.000 / 35.022 | 4.500 / 4.489 | 0.002 / 0.000 |
| 25 mm/s | 24.987 / 25.000 / 25.013 | 6.100 / 6.084 | 0.001 / 0.000 |

另一次用第 21 课原播放器分 4 段对照，从横移反馈帧算出的速率约在 1.24～52.57 mm/s 之间。帧间差分覆盖一个短区间，所以在解析曲线的零速路点附近也可能读到小的正数。

可自行把巡航速度改成 `28.0`。先预测经过中间点时的速度和横移总时长，再到 B 核对。所有结果属于 RViz2 使用的运动学模型。
