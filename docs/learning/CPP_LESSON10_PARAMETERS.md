# C++ 第十课 · 让正在运行的方块变色、变大

你在第九课把 RGB 常量改成红色，重新编译后看到方块改变。这次试着让 RViz2 一直开着，把眼前的绿色方块调成蓝色，再把边长从 25 mm 改成 50 mm。两个终端都用电脑 WSL Ubuntu 22.04 / ROS 2 Humble。本课用本机通信域 18，与第九课用的域 17 分开。

## 先动手，观察每次修改

**终端 A · 首次编译并启动。** 打开电脑 WSL，输入：

```bash
# 进入工作区；当前终端使用电脑的 Humble。
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash

# 首次或修改 C++、Launch 文件后编译安装。
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --packages-up-to jixiebi_learning \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3

# 构建成功后加载安装目录，打开发布节点与 RViz2，保持 A 运行。
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=18
ros2 launch jixiebi_learning lesson10_marker.launch.xml
```

窗口中首先是绿色方块，边长为半个网格。以后使用相同构建，加载环境并运行最后的 Launch 命令即可。结束整节实验时在 A 按 Ctrl+C，一起退出两个进程。

**终端 B · 读值并在线修改。** 另开一个同发行版的 WSL 终端。在每条 `set` 之后看一眼 RViz2，节点约每秒画一帧。

```bash
# 使用同一工作区及通信域；这里无需编译。
source /opt/ros/humble/setup.bash
source $HOME/Jixiebi/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=18

# 先读发布节点当前的颜色和边长，应是 green、0.025。
ros2 param get /jixiebi_lesson10_marker color
ros2 param get /jixiebi_lesson10_marker side_m

# 第一次修改只变颜色，几何尺寸保持原值。
ros2 param set /jixiebi_lesson10_marker color blue

# 看见蓝色后再调边长。0.05 m 是 50 mm，每条边从半格变成一格。
ros2 param set /jixiebi_lesson10_marker side_m 0.05
```

最后试一个节点应当拒绝的值。允许的颜色为 `red`、`green`、`blue`；示例边长为 **0.01～0.10 m，含两端**。例如 0.5 m 超出这个显示范围。

```bash
# 应看到设置失败及原因，重新读取仍为 0.05；方块仍是蓝色。
ros2 param set /jixiebi_lesson10_marker side_m 0.5
ros2 param get /jixiebi_lesson10_marker side_m
```

## 跟一条 blue 请求读三个位置

在 WSL 打开 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/src/lesson10_marker_parameters.cpp`。到这里，第九课的发布循环只多了一条输入途径。终端 B 把请求交给**发布节点的参数服务**，RViz2 仍然只订阅 **Marker 话题**。[完整中文注释源码](../../ros2_ws/src/jixiebi_learning/src/lesson10_marker_parameters.cpp)

**① 构造函数声明节点公开的参数。** `declare_parameter` 给出名字、类型与默认值。颜色用字符串，边长用以米为单位的浮点数。登记回调的句柄保存为成员，使节点整个运行期间都能检查修改。

```cpp
// 先登记校验函数，启动参数与以后的修改都由它检查。
validation_handle_ = add_on_set_parameters_callback(validate_parameters);
declare_parameter<std::string>("color", "green");
declare_parameter<double>("side_m", 0.025);
```

**② 参数服务收到请求，调用 `validate_parameters()`。** 函数收到包含候选值的列表。对 `blue`，它检查这是允许的颜色；对 `side_m`，它检查数值有限且落在范围内。它把是否接受与原因放在 `SetParametersResult` 中返回。这里先校验，交给 ROS 提交，避免后面拒绝请求时画面已被改了一半。终端 B 的两条 `set` 命令是两次独立请求。[ROS 2 Humble 参数说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Basic/About-Parameters.rst)

**③ 下一次定时回调读取已提交的参数，生成 Marker。** 对于 `color=blue`，蓝色通道取 1，红、绿通道取 0。将 `side_m` 设为 0.05 时，代码同时将方块中心升到 0.025 m，所以放大后底面还贴着网格。

```cpp
// 以下各行来自 publish_block()，省略了与第九课相同的设置。
const auto color = get_parameter("color").as_string();
const double side_m = get_parameter("side_m").as_double();
// ...... 创建 marker，设置坐标系和标识 ......
marker.scale.x = side_m;              // y、z 的边长也等于 side_m。
marker.pose.position.z = side_m / 2.0;
marker.color.b = color == "blue" ? 1.0f : 0.0f;
publisher_->publish(marker);          // 发给 /jixiebi/learning/block_marker 的订阅者。
```

## Launch 做了哪一步

WSL 文件是 `$HOME/Jixiebi/ros2_ws/src/jixiebi_learning/launch/lesson10_marker.launch.xml`。第一个 `<node>` 启动 C++ 发布器，`<param>` 将启动参数传入节点；第二个 `<node>` 启动 RViz2 并打开第九课的网格和视角。一条 Launch 命令就组织好了这两个进程。[本课 Launch 文件](../../ros2_ws/src/jixiebi_learning/launch/lesson10_marker.launch.xml)

在线修改只对当前节点实例生效。若想**下次启动就看到红色、40 mm 的方块**，先在 A 按 Ctrl+C，然后在原终端运行：

```bash
# 指定这次启动的初值；可在 B 再改。
ros2 launch jixiebi_learning lesson10_marker.launch.xml color:=red side_m:=0.04
```

如果 B 提示找不到节点，先确认 A 仍在运行，再核对两个终端的 `ROS_DOMAIN_ID` 都是 18。若窗口上方有 `Global Status: Warn` 但方块已经显示，可以像第九课一样直接观察变化；本课 Marker 与视图仍然使用相同的固定坐标系。

**自己试一遍**：让方块从 50 mm 缩到 20 mm。先算出新的中心高度，再在 B 改一个参数。能说清“哪一段代码保证它缩小后仍贴住网格”，这节课就学到了。
