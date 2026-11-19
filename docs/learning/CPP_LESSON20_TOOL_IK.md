# 第二十课 · 给工具一个坐标，让模型自己算关节角

目标是让 RViz2 中黄色球心到达第十九课读过的坐标。上节的输入是关节角，输出是位置；这节输入位置与工具俯仰，输出四个关节角。全部操作在电脑 WSL 的 ROS 2 Humble 中完成。

## 1 · 先定义题目

在 WSL 打开 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/lesson20_tool_ik.cpp`。从 `run()` 跟到同目录 `lesson_ik.hpp` 的 `read_geometry()`、`solve()` 和 `check_solution()`，再回到 `run()` 看轨迹发送。[中文注释入口](../../ros2_ws/src/jixiebi_learning/src/lesson20_tool_ik.cpp) · [第二十和二十一课共用的逆解](../../ros2_ws/src/jixiebi_learning/src/lesson_ik.hpp)

- **位置**以 `base_link` 为参考，零姿向前是 +Y，向上是 +Z。目标为 **(0.108, 45.991, 530.987) mm**，代表 `tool_tip` 的原点，即黄色球心。
- **姿态**用 `pitch_deg` 指定工具杆在肩肘运动平面里的方向。水平向前为 0°，竖直向上为 90°。在上一课的目标姿态里，肩、肘、腕角相加为 89.73° − 10° + 0.27° = **80°**。它不是腕关节自身的角。
- **模型**由 `lesson_model::read_chain()` 从正在运行的 URDF 取得。沿用肩轴高 170 mm、上臂 105 mm、前臂 75 mm、工具段 185 mm，计算内部均用米和弧度。数据依据可查[事实表](ARM_FACTS.md)和[模型 Xacro](../../ros2_ws/src/jixiebi_learning/urdf/lesson12_arm.urdf.xacro)。

## 2 · 写出几何逆解

在 `solve()` 里先看底座。对于目标位于手臂朝向前方的一支，水平方向向量为 `(−sin(yaw), cos(yaw))`，所以写成 `atan2(−x, y)`。正上方的点没有水平方向，此时保留当前底座角。

再看两个平面坐标：`r` 是腕轴沿手臂水平朝向的投影，`z` 是腕轴比肩轴高多少。用目标减掉工具杆的投影，就把“球心在哪里”变成“腕轴在哪里”。下面是代码中依次用到的式子，`side` 在前向分支取 +1、反向取 −1。

```cpp
// 输入长度都以米表示。球心退回一个工具长度，就到腕轴。
const double r = side * radius - g.tool * std::cos(pitch);
const double z = target.z() - g.height - g.tool * std::sin(pitch);
// 上臂、前臂、肩到腕的直线距离构成三角形，由余弦定理求肘角。
const double cos_e = (r*r + z*z - g.upper*g.upper - g.forearm*g.forearm)
                     / (2.0 * g.upper * g.forearm);
```

这个例子的腕轴相对肩轴约为向前 **13.866 mm**、向上 **178.798 mm**。若 `cos_e` 超出 [−1, 1] 且超过舍入误差，这个三角形就拼不出来。可达时 `+acos(cos_e)` 和 `−acos(cos_e)` 都要考察。

```cpp
// 肩角等于“肩指向腕的角度”减去上臂相对这条连线的偏角。
const double shoulder = wrap(std::atan2(z, r) -
    std::atan2(g.forearm * std::sin(elbow), g.upper + g.forearm * std::cos(elbow)));
// 三个俯仰关节的角度相加得到工具俯仰，剩余角度分配给腕。
Angles q{base, shoulder, elbow, wrap(pitch - shoulder - elbow)};
```

例如同一个默认目标可以对应肘约 **−10° 或 +10°**。后一支的肩约 81.40°、腕约 −11.40°，一样够得到。程序先筛掉超出本模型 ±90° 限位的解，再比较各关节距当前角的变化量平方和。上一课的 0° 或 −5° 起点会选到负肘解。[平面两连杆几何逆解示意与讲解](https://modernrobotics.northwestern.edu/nu-gm-book-resource/inverse-kinematics-of-open-chains/)

## 3 · 看算出的角如何进入 RViz2

`run()` 把所选的四个角送回 KDL 正运动学，用毫米比较球心位置，并检查工具杆方向。通过后，把当前模型角写为轨迹首点，逆解写为末点；第十八课的 Action 播放器插值并发布 JointState，RViz2 通过模型 TF 显示运动。插值发生在关节空间，因此球心在两个点之间可以走弧线。

- `IK_DEG` 是所选解；`FK_ERROR_MM` 表示这组角在同一份模型里回算的终点误差。
- `RESULT=SUCCEEDED` 是模型播放器完成。随后用第十九课从新的一帧状态再读一次球心，可以核对实际显示的模型姿态。这个误差只描述数学模型。
- `ALREADY_AT_TARGET` 表示模型已经处在选中的关节解。`NO_IK_SOLUTION` 表示给定俯仰和限位下没有可用解，该次保持原姿态。

## 4 · 在 WSL 用两个终端操作

自己改过源码时，在 WSL 终端 B 编译一次：

```bash
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to jixiebi_learning
```

两个新 WSL 终端各执行一次环境设置。若上一课的终端 A 正用域 41 运行 `checked_action`，直接保留它，只在 B 加载环境。

```bash
# 加载 Humble 和自己的程序。两端使用上一课的本机通信设置。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=41 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**WSL 终端 A** 启动模型，等“起点核对已开启”，保持运行：

```bash
ros2 launch jixiebi_learning lesson16_playback.launch.py input:=checked_action
```

**WSL 终端 B** 给坐标、看模型弯肘、再读终点。命令中的小数点让 ROS 2 将参数解释为 double。

```bash
# 输入黄色球心的目标毫米坐标和工具俯仰角。
ros2 run jixiebi_learning lesson20_tool_ik --ros-args \
  -p x_mm:=0.108 -p y_mm:=45.991 -p z_mm:=530.987 -p pitch_deg:=80.0

# 等上一步完成，从最新 JointState 独立回读显示的位置。
ros2 run jixiebi_learning lesson19_tool_position
```

从上一课的 −5° 或新启动的 0° 出发，应看到肩约 89.729°，肘约 −9.998°，腕约 +0.268°，回读 `XYZ_MM=[0.108, 45.991, 530.987]`。目标只保留了三位小数，所以倒算出来的肘角也会和 −10° 稍有差异。

**亲手改一次**：把目标换成第十九课 −5° 的位置，`x_mm:=0.055 y_mm:=23.507 z_mm:=533.978 pitch_deg:=85.0`。先判断肘会往哪边转，再看 `IK_DEG` 和回读的 XYZ。
