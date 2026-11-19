# C++ 第九课 · 用 RViz2 看见自己发布的方块

目标是在电脑上画出一块边长 25 mm 的绿色方块。终端 A 运行你写的 C++ 发布节点，终端 B 运行 RViz2。两边通过 ROS 2 Topic 通信，全课都在电脑 WSL Ubuntu 22.04 / ROS 2 Humble 完成。

本地源码已保留最后的改色练习（红色）。下面用初始的绿色版本讲解。想在节点运行时切换颜色，接着看[第十课动态参数](CPP_LESSON10_PARAMETERS.md)。

## 打开哪个文件

在 WSL 编辑 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/lesson09_marker_publisher.cpp`。课程代码已放好，可以对照带中文注释的[完整 C++ 源码](../../ros2_ws/src/jixiebi_learning/src/lesson09_marker_publisher.cpp)逐段读或写。ROS 已经定义了 `visualization_msgs/msg/Marker`，本课直接使用它。

另两个位置决定代码如何运行。包的 [CMakeLists.txt](../../ros2_ws/src/jixiebi_learning/CMakeLists.txt) 用 `add_executable` 把这个 cpp 编译为 `lesson09_marker_publisher`，将它链接到 ROS 依赖并安装。包内的 [lesson09_marker.rviz](../../ros2_ws/src/jixiebi_learning/rviz/lesson09_marker.rviz) 保存 RViz2 要订阅的话题、固定坐标系、网格和观察角度。

## 按实际调用顺序读 C++

**① 从 main 找到谁会调用发布函数。** `main()` 先初始化 ROS，再构造 `BlockMarkerPublisher` 对象。构造函数里这三行建立话题并登记定时器：

```cpp
// 保存最近一条消息；稍后打开且使用相同持久性策略的 RViz2 可以收到它。
auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
publisher_ = create_publisher<Marker>("/jixiebi/learning/block_marker", qos);
// 每隔 1 秒，回调使用当前节点对象调用它的成员函数。
timer_ = create_wall_timer(1s, [this]() { publish_block(); });
```

`publisher_` 和 `timer_` 是类成员，构造函数结束后仍然存在。`main()` 中的 `rclcpp::spin(...)` 等待事件；定时器到期时，执行器调用上面登记的回调，然后进入 `publish_block()`。RViz2 加入话题或关闭窗口都不影响这个定时器。

**② 说明要画什么、在哪个坐标系里画。** 在 `publish_block()` 中创建一条新的消息，并填写：

```cpp
Marker marker;
marker.header.frame_id = "lesson09_world";  // 下面的中心坐标相对于本课世界原点。
marker.header.stamp = now();                // 使用这次更新的 ROS 时间。
marker.ns = "blocks";                       // 同一话题上，ns 与 id 一起标识图形。
marker.id = 0;
marker.type = Marker::CUBE;                 // 立方体。
marker.action = Marker::ADD;                // 首次创建，以后更新同一个图形。
```

这里的 `action` 是 Marker 消息中的一个绘图字段。第八课的 ROS Action 才有目标、反馈、取消和结果。这个节点只用 Topic，RViz2 收到消息后根据字段绘图。[Humble 标准消息定义](https://github.com/ros2/common_interfaces/blob/humble/visualization_msgs/msg/Marker.msg)

**③ 算中心，填尺寸、朝向和颜色，然后发送。** RViz2 用米表示位置与尺寸。`scale` 给出方块沿三个轴的完整边长，`pose.position` 给出中心。网格在 `z=0`，要让底面贴住网格，中心应位于 `0.025 / 2 = 0.0125` m。若将中心设成零，方块就会一半在网格上方、一半在下方。

```cpp
constexpr double side_m = 0.025;       // 25 mm = 0.025 m
marker.scale.x = side_m;
marker.scale.y = side_m;
marker.scale.z = side_m;
marker.pose.position.x = 0.20;         // 中心沿世界坐标系的 +X 方向 20 cm。
marker.pose.position.y = 0.0;
marker.pose.position.z = side_m / 2.0;
marker.pose.orientation.w = 1.0;       // 默认 x、y、z 为零，表示没有旋转。
marker.color.r = 0.0f;
marker.color.g = 1.0f;                 // 纯绿色。
marker.color.b = 0.0f;
marker.color.a = 1.0f;                 // 完全不透明；0 表示完全透明。
publisher_->publish(marker);           // 把填好的消息发送给该话题的订阅者。
```

## 终端 A · 编译并启动发布节点

打开电脑上的 WSL Ubuntu 22.04。若第八课的程序还在前台，可以先按 Ctrl+C 取回提示符。

```bash
# 进入工作区并加载电脑上的 ROS 2 Humble。
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash

# 首次或改动 C++ 后编译；生成程序也会随包安装。
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-up-to jixiebi_learning \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3

# 看到 Summary: 2 packages finished 后，加载工作区，让 ros2 run 能找到程序。
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=17
ros2 run jixiebi_learning lesson09_marker_publisher
```

看到“方块发布器就绪”后保持 A 打开。启动信息只打印一次，后续每秒继续发布。下次使用同一份构建可以跳过 `colcon build`。

## 终端 B · 打开 RViz2 看方块

再开一个**同一发行版的 WSL 终端**，使用与 A 一致的本机通信设置。

```bash
# 新终端也需要加载 ROS 和工作区。
source /opt/ros/humble/setup.bash
source $HOME/Jixiebi/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=17

# 从已安装的包找到本课视图配置，启动图形窗口。
rviz2 -d "$(ros2 pkg prefix --share jixiebi_learning)/rviz/lesson09_marker.rviz"
```

在窗口左侧的 **Displays** 里展开 **Block Marker**，它订阅 `/jixiebi/learning/block_marker`。场景中央应有一个绿色立方体，网格每格 5 cm，方块的一条边占半格。**Global Options → Fixed Frame** 和消息的 `header.frame_id` 都是 `lesson09_world`，所以同一坐标系内就能画出它。这个练习的 `+X`、原点和网格都由示例定义，与真机桌面的位置关系要靠后续标定建立。

本课在 WSLg 实际打开时，左栏显示黄色 `Global Status: Warn`，同时右侧方块正常可见。本例没有单独发布 TF 树，Marker 已经使用固定坐标系；看到这个画面即可继续。若改用空白 RViz 窗口，可手工设置 **Fixed Frame**，再点 **Add → By topic**，选择该话题下的 **Marker**。如果窗口中只有网格，先核对固定坐标系和 Marker 的话题。也可以退出 B 的 RViz，在 B 查看一条消息，再用上一条命令打开视图：

```bash
# 核对 frame_id、scale.x、color.g 与 color.a，最多等 5 秒。
timeout 5s ros2 topic echo /jixiebi/learning/block_marker --once
```

## 改两行就能自测

在 C++ 文件中将 `marker.color.r` 从 `0.0f` 改为 `1.0f`，将 `marker.color.g` 从 `1.0f` 改为 `0.0f`。保持 B 打开，在 A 中按 Ctrl+C，重新编译并启动节点。若原位置的同一个方块变红，你就实际验证了从 C++ 字段、ROS 话题到 RViz 图形的完整链路。结束时两个终端分别按 Ctrl+C。
