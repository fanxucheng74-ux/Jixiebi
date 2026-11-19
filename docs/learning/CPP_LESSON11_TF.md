# C++ 第十一课 · 用 tf2 看工具相对谁在动

上节课你算出边长 20 mm 的方块贴着网格时，中心高 0.01 m。这个高度是相对网格说的。本课换两个观察基准，看看同一个工具为什么能同时有两个不同的高度。全程在电脑 WSL Ubuntu 22.04 / ROS 2 Humble，坐标取自教学示例。

## 在纸上画三个原点

```text
lesson11_world   桌面参考系，网格位于它的 XY 平面
  └─ lesson11_base   底座：沿 world 的 +X 偏 10 cm，沿 +Z 抬高 5 cm，轴朝向相同
       └─ lesson11_tool   工具：在 base 的 XY 平面内绕圈，半径 10 cm，同时绕 +Z 转向
```

**父坐标系提供尺子和方向，子坐标系是我们摆放的那组轴。** `header.frame_id` 写父系，`child_frame_id` 写子系。`translation` 描述子系原点在父系中的坐标，`rotation` 描述子系的轴相对父系转了多少。底座既是世界的子系，也是工具的父系。

假设工具转到正对底座 **+Y** 的那一刻。站在底座系看，工具原点是 `(0, 0.10, 0)` m；站在世界系看，它是 `(0.10, 0.10, 0.05)` m。这里底座和世界轴平行，所以平移可以直接相加。一般情况要先按父子姿态旋转向量，再加平移；跨级查询时 tf2 会沿树完成这个计算。

## 对着 C++ 读两条边

在 WSL 写入或打开 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/lesson11_tf_broadcaster.cpp`。文件已备好。[带中文注释的完整 C++](../../ros2_ws/src/jixiebi_learning/src/lesson11_tf_broadcaster.cpp)

**第一条边在构造时发一次。** `main()` 创建节点时，构造函数调用 `publish_base()`。以下是函数中的关键语句。

```cpp
base.header.frame_id = "lesson11_world";   // 用世界系的尺子和方向描述底座。
base.child_frame_id = "lesson11_base";
base.header.stamp = now();
base.transform.translation.x = 0.10;       // x=0.10、y=0、z=0.05，单位为米。
base.transform.translation.z = 0.05;
base.transform.rotation.w = 1.0;           // 其余分量默认 0，两个坐标系朝向一致。
static_broadcaster_.sendTransform(base);   // 经 /tf_static 广播。
```

此后底座位置固定。静态广播器保存在节点成员中；节点保持运行时，稍后才打开的 RViz2 或 `tf2_echo` 也能收到保留的静态变换。

**第二条边由定时器持续更新。** 构造函数还注册了一个 50 ms 定时器。`main()` 里的 `rclcpp::spin` 调度它调用 `publish_tool()`，约每秒 20 次。该函数取一次 `now()`，用同一时刻算出角度并填写时间戳。

```cpp
const double theta = angular_speed_rad_s * (stamp - start_time_).seconds();
// theta 单位为弧度；角速度 0.5 rad/s，半径 radius_m 为 0.10 m。
tool.header.frame_id = "lesson11_base";
tool.child_frame_id = "lesson11_tool";
tool.header.stamp = stamp;
tool.transform.translation.x = radius_m * std::cos(theta);  // 圆周上的 x、y，z 默认为 0。
tool.transform.translation.y = radius_m * std::sin(theta);
tool.transform.rotation.z = std::sin(theta / 2.0);          // 下面两行控制轴的朝向。
tool.transform.rotation.w = std::cos(theta / 2.0);
dynamic_broadcaster_.sendTransform(tool);                   // 经 /tf 广播带时间戳的样本。
```

位置和朝向可以独立设置。两行 `translation` 让工具的**原点**绕圈；两行 `rotation` 让工具的**红色 +X 轴**始终指向圆外。绕 Z 转 `theta` 时，单位四元数的 `(x,y,z,w)` 为 `(0,0,sin(theta/2),cos(theta/2))`。只想读角度时，可以看查询结果中的 `RPY (degree)`，第三个数是 yaw。示例一圈约 12.6 秒。

## 终端 A · 编译，一次启动广播节点和 RViz2

打开 **电脑 WSL**。首次或修改源码后编译；已有本课构建时可以跳过 `colcon build`。

```bash
# 进入工作区，加载 WSL 的 ROS 2 Humble。
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash

# 编译学习包和它的工作区依赖，同时安装 Launch 与 RViz 视图。
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-up-to jixiebi_learning \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3

# 构建成功后，加载工作区；本课两个终端都用本机域 19。
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=19
ros2 launch jixiebi_learning lesson11_tf.launch.xml
```

保持 A 运行。RViz2 的 **Fixed Frame** 是 `lesson11_world`，左栏已打开 **TF** 显示。看三个名字旁的坐标轴：红、绿、蓝分别指 **+X、+Y、+Z**。世界和底座两组轴固定，工具那组轴的原点和红轴方向都在变。展开左侧 **TF → Tree** 可以按名字核对父子层级。[Launch 文件](../../ros2_ws/src/jixiebi_learning/launch/lesson11_tf.launch.xml) · [RViz2 显示配置](../../ros2_ws/src/jixiebi_learning/rviz/lesson11_tf.rviz)

## 终端 B · 先查一条边，再查整条链

另开一个**同一发行版的电脑 WSL**，加载与 A 一样的环境。

```bash
source /opt/ros/humble/setup.bash
source $HOME/Jixiebi/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=19
```

`tf2_echo A B` 可以读成“**以 A 为参考，B 的原点和轴在哪里？**” 第一项 `Translation` 就是 B 原点用 A 表示的坐标。若要把 B 系的一整个点换到 A 系，该变换同样适用。以下查询**逐条**运行；每条看两三次输出，在 **B 按 Ctrl+C** 再输下一条。A 和 RViz2 可以一直开着。[Humble 的 tf2_echo 源码和参数说明](https://github.com/ros2/geometry2/blob/humble/tf2_ros/src/tf2_echo.cpp)

```bash
# ① 世界看底座：固定为 [0.100, 0.000, 0.050]，四元数 [0, 0, 0, 1]。
ros2 run tf2_ros tf2_echo lesson11_world lesson11_base
```

```bash
# ② 底座看工具：x、y 在变，x²+y² 约为 0.10²，z 为 0。
ros2 run tf2_ros tf2_echo lesson11_base lesson11_tool
```

```bash
# ③ 世界看工具：tf2 组合前两条边。圆心 x=0.10，工具高度 z=0.05。
ros2 run tf2_ros tf2_echo lesson11_world lesson11_tool
```

这次实测，两路查询在同一时间戳 `1795003487.974552770` 分别输出 `[0.075, 0.066, 0.000]` 和 `[0.175, 0.066, 0.050]` m。工具相同，换了观察基准，x 增加 0.10 m，z 增加 0.05 m。你逐条查询时工具一直在动，因此比较两次输出时优先看高度和圆周范围；要逐项相减，需要对齐时间戳。

查询刚启动时可能先显示 `Waiting for transform`，等几秒看后续输出；若始终无数据，核对 A 还在运行、两个终端的通信域都是 19，frame 名字与源码一致。全部结束后在 A 按 Ctrl+C。

**自己验证方向**：把第一条查询的两个名字对调，先预测平移，再运行。底座和世界的轴朝向一致，此例应该得到 `[-0.100, 0.000, -0.050]` m。若两个系之间还有旋转，求逆时也要处理旋转。
