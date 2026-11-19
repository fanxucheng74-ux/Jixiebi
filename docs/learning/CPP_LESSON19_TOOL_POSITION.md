# 第十九课 · 从四个关节角算工具端 XYZ

上一课你能让模型从任意已显示的肘角接着弯。这一课问：肘从 0° 弯到 −10°，RViz2 中的黄色球心移动了多少毫米？

给定各关节角，通过每一段的安装位置和转轴求工具端在底座系中的位置，这叫**正运动学**（forward kinematics，FK）。所求点是 `tool_tip` 原点，即黄色球心。所有操作都在 **电脑 WSL / ROS 2 Humble**，RK 可以关机。数据来自[机械臂事实索引](ARM_FACTS.md)和[第十二课 Xacro](../../ros2_ws/src/jixiebi_learning/urdf/lesson12_arm.urdf.xacro)。

## 第一步 · 从正在显示的模型取出一条链

在 WSL 打开 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/lesson19_tool_position.cpp`。从 `show_position()` 进入，旁边的 `lesson_model.hpp` 包含两课共用的 `read_chain()` 和 `angles_in_chain_order()`。[第十九课入口](../../ros2_ws/src/jixiebi_learning/src/lesson19_tool_position.cpp) · [模型读取函数](../../ros2_ws/src/jixiebi_learning/src/lesson_model.hpp)

A 窗口里的状态发布器已经把模型描述发到 `/jixiebi/learning/follow/robot_description`。B 用 `transient_local` 读取它保存的 URDF。这样可以直接用运行中的模型，无需在 C++ 再填一遍尺寸。

```cpp
// 树保存整个机械臂；从树里取出底座到工具点经过的那一条链。
KDL::Tree tree;
KDL::Chain chain;
if (!kdl_parser::treeFromString(urdf.data, tree)) throw std::runtime_error("URDF 解析失败");
if (!tree.getChain("base_link", "tool_tip", chain)) {
    throw std::runtime_error("模型中找不到 base_link 到 tool_tip 的链");
}
```

链的顺序是 `base_link → yaw_link → upper_arm → forearm → wrist → tool_tip`。共五段，只有四个会转的关节。最后的 `tool_tip_fixed` 把工具点放在腕关节外 185 mm；这一段始终参与位置计算。

## 第二步 · 把一帧关节角填进 q

`show_position()` 从 `/jixiebi/learning/follow/joint_states` 读取一帧本机模型状态，检查时间戳。随后 `angles_in_chain_order()` 沿链逐段读取关节名，在消息的 `state.name` 内找到对应列，再取同列的 `state.position`。本模型的四个活动关节都是转轴，位置单位为 rad。[Humble JointState 字段定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg)

```cpp
// 以下是循环里的关键几行。完整函数还会拒绝角度缺失、名称重复和非有限数。
if (joint.getType() == KDL::Joint::Fixed) continue;
const auto &name = joint.getName();
const auto it = std::find(state.name.begin(), state.name.end(), name);
const double rad = state.position[std::distance(state.name.begin(), it)];
q(column++) = rad;  // 输出列按照运动链的活动关节顺序，而输入列按名字查找。
```

这里的 `continue` 只跳过“填入一个角度”。固定段依然在 `chain` 里，所以黄色球心不会被错算成腕关节中心。

## 第三步 · 算工具相对底座的位置

```cpp
KDL::ChainFkSolverPos_recursive fk(chain);  // 使用 URDF 里的关节轴和安装变换。
KDL::Frame base_to_tip;
if (fk.JntToCart(q, base_to_tip) < 0) throw std::runtime_error("正运动学计算失败");
const auto &p = base_to_tip.p;             // p.x()、p.y()、p.z() 单位为米。
```

可以把这一步读成：“站在底座原点，把肩、肘、腕依次转到 q 指定的角度，沿转过后的每一段走到工具点。” KDL 依序复合各段的变换。`Frame` 同时有位置和朝向，本课只取位置 `p`，在打印时乘 1000 转成 mm。[KDL 逐段计算的实现](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/chainfksolverpos_recursive.cpp)

## 第四步 · WSL 两个终端运行

首次克隆或修改 C++ 后，先在 WSL 编译一次：

```bash
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to jixiebi_learning
```

**电脑 WSL，终端 A、B 各执行一次**。上一课的模型窗口仍在运行时，在那个终端按 Ctrl+C，等提示符出现后把它当作 A。本机验证时上一课的 UDP 配置未收到模型描述，本课在两个终端使用同一套本机设置。

```bash
# 加载 Humble 和工作区；清除上一课的专用 UDP 配置，让本机两个终端使用 Fast DDS 默认传输。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=41 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**WSL 终端 A** 启动一份模型，保持窗口运行：

```bash
# 用上一课的启动入口加载同一份模型。等 A 显示“起点核对已开启”。
ros2 launch jixiebi_learning lesson16_playback.launch.py input:=checked_action
```

**WSL 终端 B** 先读取一个位置，再移动模型并读取第二个位置：

```bash
# 新启动时肘角为 0°。第一次读数输出每个关节的 rad 和工具端在底座系中的 XYZ_MM。
ros2 run jixiebi_learning lesson19_tool_position

# 复用上一课的 Action 让模型弯到 −10°。等 RESULT=SUCCEEDED。
ros2 run jixiebi_learning lesson18_from_state --ros-args -p elbow_target_deg:=-10.0

# 第二次读数。两次使用同一个 C++ 程序，变化来自新收到的关节角。
ros2 run jixiebi_learning lesson19_tool_position
```

| 参考姿态，只有肘角改变 | X（mm） | Y（mm） | Z（mm） |
|---|---:|---:|---:|
| 肘角 0° | 0.002 | 0.848 | 534.998 |
| 肘角 −10° | 0.108 | 45.991 | 530.987 |
| 后一次减前一次 | +0.106 | +45.143 | −4.011 |

为什么主要沿 +Y 移动？看第十二课模型：肘下游有 75 mm 前臂和 185 mm 工具段，合计约 260 mm；它们开始时接近竖直。粗算水平位移是 `260 × sin(10°) ≈ 45.1 mm`，下降量是 `260 × (1 − cos(10°)) ≈ 4.0 mm`。C++ 用完整四轴模型，保留了起始底座 −0.135°、肩 89.730°、腕 +0.270°，因此初始的 Y 也略大于零。

如果 B 提示缺少模型描述，核对 A 的 launch 含有 `input:=checked_action` 并检查两窗的域和回环设置。看完在 A 按 Ctrl+C；B 的查询程序每次打印后自行退出。

**自己改一次**：在 B 把目标改成 `elbow_target_deg:=-5.0`。先预测 Y 和 Z 相对 0° 的变化，再运行 `lesson19_tool_position` 对照。能解释为什么水平位移接近原来的一半，而下降量接近原来的四分之一，就理解了关节角与末端位置的关系。

下一课把这组坐标用作输入，用几何逆解求出关节角。[第二十课](CPP_LESSON20_TOOL_IK.md)
