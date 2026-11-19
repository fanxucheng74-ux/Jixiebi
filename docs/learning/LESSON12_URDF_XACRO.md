# 第十二课 · 写一条会弯肘的机械臂

上节课的逆变换你算对了：底座与世界的坐标轴同向，站在底座看世界原点，平移为 `(-0.10, 0, -0.05)` m。这节课把连杆装在坐标系上，让 ROS 根据**关节角**计算每一段的位姿。全课在电脑 WSL Ubuntu 22.04 / ROS 2 Humble 上操作。

## 先看四个滑块控制的结构

```text
base_link → yaw_link → upper_arm → forearm → wrist → tool_tip
          底座转向     肩俯仰      肘弯曲    腕俯仰   固定连接
```

底座是灰色，上臂蓝色，前臂绿色，腕部到工具点用橙色杆表示，黄色球心标出末端。模型沿用项目既有的四关节标称几何，模拟角度用弧度，长度用米。滑块范围为教学演示所选；真机舵机的 P 值与关节角还需分别标定。本模型只显示运动学，暂未建模夹爪开合、负载与碰撞。

## 三个文件各管什么

下面三个文件都在 WSL 的 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/` 下。

| 包内文件 | 阅读重点 |
| --- | --- |
| [urdf/lesson12_arm.urdf.xacro](../../ros2_ws/src/jixiebi_learning/urdf/lesson12_arm.urdf.xacro) | 你写模型的地方。属性保存肩高和臂长，宏生成重复的杆件，关节声明父子关系、转轴与安装位置。 |
| [launch/lesson12_arm.launch.py](../../ros2_ws/src/jixiebi_learning/launch/lesson12_arm.launch.py) | 读取 Xacro，展开为 URDF 文本，把文本作为 `robot_description` 参数交给状态发布器，同时打开 RViz2。这里的 Python 只负责启动。 |
| [rviz/lesson12_arm.rviz](../../ros2_ws/src/jixiebi_learning/rviz/lesson12_arm.rviz) | 保存视角、网格及 RobotModel 显示设置。固定参考系为 `base_link`，模型描述话题为 `/robot_description`。 |

安装由包内 [CMakeLists.txt](../../ros2_ws/src/jixiebi_learning/CMakeLists.txt) 负责，它将 `urdf/`、`launch/`、`rviz/` 安装到工作区。`robot_state_publisher` 和滑块界面 `joint_state_publisher_gui` 使用 ROS 提供的程序。

## 对着源码写：Xacro 展开什么，URDF 描述什么

**1. 定义长度，再生成连杆。** Xacro 可以做变量替换、运算和宏展开。下面这两行分别来自模型开头和连杆定义处。把长度集中到同一处，连杆图形和肘关节安装位置就能共用同一个数。[ROS 2 Humble 的 Xacro 教程](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.rst)

```xml
<!-- 标称上臂长 105 mm。rod 宏展开成一个有颜色和形状的 URDF link。 -->
<xacro:property name="upper_length" value="0.105"/>
<xacro:rod name="upper_arm" length="${upper_length}" width="0.027" material="blue"/>
```

打开文件内的 `rod` 宏，看 `<visual><origin xyz="0 ${length/2} 0"/>`。连杆坐标系设在近端关节，方盒几何默认以自身中心为原点，因此图形沿 +Y 偏移**半根杆长**，两端刚好落在近端与远端。

若想亲眼看宏展开的结果，在已加载 Humble 的 WSL 终端运行下面一行；在分页器内输入 `/link name="upper_arm"` 并回车，按 `q` 退出。看到的标准 URDF 会直接写出半长 0.0525 m 和全长 0.105 m。

```bash
xacro $HOME/Jixiebi/ros2_ws/src/jixiebi_learning/urdf/lesson12_arm.urdf.xacro | less
```

**2. 把前臂安装到上臂末端。** 下面是模型里的肘关节，中文注释标出最容易混的两行：

```xml
<joint name="elbow_flex" type="revolute">
  <parent link="upper_arm"/><child link="forearm"/>
  <!-- 安装点用父连杆坐标表示：沿上臂 +Y 走整整一根杆，才到肘轴。 -->
  <origin xyz="0 ${upper_length} 0"/>
  <!-- 关节角绕局部 +X 旋转；在零姿，正角使前臂由 +Y 抬向 +Z。 -->
  <axis xyz="1 0 0"/>
  <limit lower="${-pi/2}" upper="${pi/2}" effort="1" velocity="1"/>
</joint>
```

`visual/origin` 定位**图形**，`joint/origin` 定位**转轴**。这解释了为什么上一段用半长、这里用全长。在 RViz2 中只弯肘，蓝色上臂应留在原处，绿色前臂、橙色腕杆和黄色球一起转动。

**3. 用模拟角度生成 TF。** 滑块从模型读出四个活动关节名。你拖动 `elbow_flex` 时，滑块节点把关节名放进 `JointState.name`，把对应的弧度值放进同一下标的 `position`，发布到 `/joint_states`。状态发布器根据 URDF 的长度、转轴和角度计算活动关节 TF。固定的 `wrist → tool_tip` 在 `/tf_static` 中保持原样，但整只手腕转动时，末端在底座系中的位置仍然会改变。[robot_state_publisher 的 Humble 接口说明](https://github.com/ros/robot_state_publisher/blob/humble/README.md)

```text
Xacro → 展开为 URDF → robot_state_publisher → /robot_description → RViz2 取得形状
滑块  → /joint_states → robot_state_publisher → /tf、/tf_static → RViz2 摆放形状
```

## 终端 A · 加载模型和 RViz2

在**电脑 WSL** 输入下面的命令。首次或编辑过模型后构建一次。若已经使用本课的最新构建，可跳过 `colcon build`。

```bash
# 进入工作区，加载电脑上的 ROS 2 Humble。
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash

# 编译工作区依赖，并把 Xacro、Launch、RViz 配置装进 install/。
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-up-to jixiebi_learning \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3

# 第十二课两个终端都用本机通信域 20。
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=20
ros2 launch jixiebi_learning lesson12_arm.launch.py
```

保持 A 打开。此时模型描述已经发布，活动关节会等 B 的第一组角度。若暂时只看到底座或 RobotModel 提示连杆缺少变换，继续启动 B。

## 终端 B · 拖动关节，同屏核对末端坐标

再开一个**同一发行版的电脑 WSL**。末尾的 `&` 把滑块窗口放到后台，这样仍可在 B 输入查询。只启动一份滑块。

```bash
source /opt/ros/humble/setup.bash
source $HOME/Jixiebi/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=20

# 打开滑块，由它持续发布模拟关节状态。
ros2 run joint_state_publisher_gui joint_state_publisher_gui &

# 先看一次消息，按名字对应 position 的下标，单位是 rad。
ros2 topic echo /joint_states --once

# 持续查看黄色末端球心在底座坐标系的位置，边拖滑块边看 Translation。
ros2 run tf2_ros tf2_echo base_link tool_tip
```

1. **先观察零姿。** 四个关节都在 0，机械臂沿 +Y 水平伸直。末端应约为 `[0, 0.365, 0.170]` m，其中 `0.365 = 0.105 + 0.075 + 0.185`。
2. **只动肘。** 将 `elbow_flex` 从中点拖到正端（约 +1.57 rad，即 90°），其余保持 0。蓝色上臂仍水平，绿色和橙色两段抬向 +Z。末端变为 `[0, 0.105, 0.430]` m：高度是肩高 `0.170` 加前臂 `0.075` 加工具长 `0.185`。
3. **换肩验证。** 先把肘归零，只把 `shoulder_pitch` 拖到正端。预测黄色球的高度，再看终端。此时连蓝色上臂也会随肩抬起，模型预测末端约为 `[0, 0, 0.535]` m。

结束时在 B 按 Ctrl+C 退出坐标查询，再关闭滑块窗口；在 A 按 Ctrl+C，一起退出状态发布器和 RViz2。若没有滑块，核对两终端都在域 20，且 A 的状态发布器仍在运行。模型角度或几何的修改若没出现，先确认改动的是 `src/` 下的 Xacro，再构建、重新加载工作区并重启 A、B。

**自己改一次模型。** 在 Xacro 开头将 `forearm_length` 从 `0.075` 改成 `0.085`，重新构建并启动。四个角度归零后，预测绿色杆会长多少、末端 y 会从 0.365 变成多少，再查 TF。找出同一个属性在哪里控制了绿色杆的长度，以及在哪里控制了腕轴的安装位置。
