# C++第六课：用rclcpp发布六轴原始状态

本机发布与订阅跑通后，继续[第七课：Service 请求与响应](CPP_LESSON07_SERVICE.md)。

本课接在第五课RK六轴顺序查询之后。只新增ROS薄适配，仍由同一个PositionReader调用同一read_arm_state；没有运动订阅/服务，不发送目标P/T。

## 1. 操作位置与本课边界

| 位置 | 做什么 |
| --- | --- |
| 电脑WSL，~/Jixiebi | 阅读/修改源码；可用本机Humble编译 |
| 电脑Windows PowerShell | 将ROS增量源码传到RK，ssh登录 |
| RK的第一个SSH窗口 | 使用Jazzy构建，启动唯一串口节点 |
| RK的第二个SSH窗口 | 只订阅话题、查看节点/类型，不打开串口 |

作者验证环境为WSL22.04/Humble与RK24.04/Jazzy；读者应使用与系统匹配的ROS发行版。此课在同一RK上验收发布/订阅；不要求从WSL跨发行版收取RK消息，不默认跨版本DDS互通。后续正式多机开发先统一发行版/接口。ROS数据经ROS通信层传送，SSH只是登录和部署。

先学Node、Publisher、Timer、Message、spin五个概念。消息元数据和CMake作为这次可构建的参考，不需要一节背下所有辅助行。

## 2. 文件在哪里

| 文件 | 作用 |
| --- | --- |
| [ServoStateRaw.msg](../../ros2_ws/src/jixiebi_interfaces/msg/ServoStateRaw.msg) | 一个舵机的编号、状态、原始P、耗时与错误 |
| [ServoStatesRaw.msg](../../ros2_ws/src/jixiebi_interfaces/msg/ServoStatesRaw.msg) | 整批序号、ROS时间、来源、完成情况和六个样本 |
| [接口CMakeLists.txt](../../ros2_ws/src/jixiebi_interfaces/CMakeLists.txt) / [package.xml](../../ros2_ws/src/jixiebi_interfaces/package.xml) | 生成和登记消息类型 |
| [servo_state_publisher.cpp](../../ros2_ws/src/jixiebi_hardware/src/servo_state_publisher.cpp) | 节点、发布者、定时回调及main，中文说明在各代码段前 |
| [节点CMakeLists.txt](../../ros2_ws/src/jixiebi_hardware/CMakeLists.txt) / [package.xml](../../ros2_ws/src/jixiebi_hardware/package.xml) | 将原native源码静态链接到ROS节点并安装 |

全部参考实现已放在同名文件中。电脑WSL可用nano ros2_ws/src/jixiebi_hardware/src/servo_state_publisher.cpp打开；对照阅读/修改，不在尾部再追加一个main。两个新包均不替换旧Python包，运行本课节点时旧串口节点应退出。

## 3. 为什么先用原始P消息

本课话题为/jixiebi/servo_states_cpp，类型为jixiebi_interfaces/msg/ServoStatesRaw。该名字与旧Python话题分开，避免不同消息类型混用同名话题。

P尚未标定成弧度或毫米；JointState.position的标准单位是转动弧度/移动米，且有同一记录时刻语义。因此不直接把1500写成JointState的位置，待标定/时间处理后再做转换层。[官方JointState定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg)

ServoStateRaw.msg的status常量为NOT_QUERIED=0、OK=1、FAILED=2；position_p仅在OK时有效，失败/未查的0是占位。整批complete和valid_count保留第五课含义，simultaneous恒为false。stamp是节点在查询后、发布前取得的ROS时间，不是舵机时间；sequence只数本节点批次，不是串口请求编号，重启后重新计数。

msg文件是类型定义，使用#中文注释。colcon自动生成servo_states_raw.hpp等C++类型，以及用于ros2 topic echo的Python类型；不手写这些生成头文件。

## 4. main：从单次运行变为事件调度

实际节点main的主链是：

```cpp
rclcpp::init(argc, argv);  // 初始化ROS，识别--ros-args参数。
rclcpp::spin(std::make_shared<ServoStatePublisher>());
rclcpp::shutdown();       // 清理ROS资源；完整源码还有异常处理。
```

make_shared创建节点对象并返回共享所有权指针；spin持续调度事件回调，直到Ctrl+C等使ROS退出。本课在当前线程执行回调，不开启六个线程抢串口。构造函数登记定时器时不会立刻查询，spin调度到定时器才调用。

旧CLI的--port /dev/ttyUSB0是自己解析argv；本课改用ROS参数--ros-args -p port:=/dev/ttyUSB0，由init与declare_parameter配合处理。程序不会自动理解两个接口为同一个语法。

## 5. 节点类：拥有串口、发布者与定时器

```cpp
class ServoStatePublisher : public rclcpp::Node {
public:
    ServoStatePublisher() : Node("jixiebi_servo_state_cpp") { /* 见原文件 */ }
};
```

继承Node使本类可以使用ROS创建发布者、声明参数和日志等功能。构造时登记的节点名为jixiebi_servo_state_cpp；包名jixiebi_hardware、可执行程序名servo_state_publisher、话题名/jixiebi/servo_states_cpp各有不同用途。

```cpp
reader_ = std::make_unique<jixiebi::PositionReader>(port, pty_simulation);
```

reader_是类成员unique_ptr，独占管理一个PositionReader，节点结束时析构释放设备及flock。它不是每次回调重新创建的临时对象。参数port默认空，未明确指定时拒绝打开设备；period_ms/query_timeout_ms/pty_simulation都只在启动读取，标记read_only防止在线改参数却未生效。

## 6. Publisher与QoS：创建通道，尚未发布

```cpp
using RawBatch = jixiebi_interfaces::msg::ServoStatesRaw;
const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
publisher_ = create_publisher<RawBatch>("/jixiebi/servo_states_cpp", qos);
```

RawBatch是长类型名的别名；模板参数<RawBatch>说明话题消息类型。Publisher创建发布端，不会在这行自动发送。QoS是消息传递规则：可靠、最多保留一条排队样本、volatile不向晚加入者回放旧状态。可靠ROS消息传递不等于自动重发串口命令。[官方QoS说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

publisher_与timer_都保存为成员SharedPtr，让它们在节点运行期间持续存在。订阅者可以有多个，均不需要再打开硬件串口。

## 7. Timer：登记以后调用哪段代码

```cpp
timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() {
    publish_state();
});
```

[this]的lambda保存当前节点对象指针，让未来的回调能调用成员publish_state。此处是登记，不是马上执行。wall timer用墙钟间隔，不依赖仿真/clock驱动。

回调开始先cancel；成功发布后reset，从完成时重新等待period_ms（默认1000ms）。因此实际周期包含查询/发布时间，并不严格为1Hz；慢查询不会产生要追赶的任务堆积。每轴查询期限默认500ms，沿用第五课。参数均为启动配置，不需要每次运行重新编译。

## 8. 回调：已有查询→类型化消息→publish

```cpp
// *reader_取得智能指针所持有的对象，仍传给第五课的引用参数。
const auto batch = jixiebi::read_arm_state(
    *reader_, std::chrono::milliseconds(query_timeout_ms_));
```

接着新建RawBatch message，把本批元数据及六项字段逐项复制。native状态显式映射ROS常量，不依赖两边枚举数值碰巧一致；只有OK复制P，其他保持新消息默认0并携带非OK状态，绝不沿用上一批P。实际发布发生在：

```cpp
publisher_->publish(message);
```

箭头->通过智能指针访问发布者对象的方法。消息序列化和ROS传输由ROS处理，不用我们手工拼网络报文；串口ASCII组帧仍在原C层。

查询失败时先发布不完整批次，再暂停定时器，节点和锁保留直到用户Ctrl+C退出；不自动重连、补查或发运动。volatile意味着晚启动的echo可能错过唯一的失败批次，应先看节点日志。Ctrl+C可能等待当前同步、有界六轴查询结束（默认最多约六次500ms加系统开销），不承诺立即中断，也不把关串口叫作已验证物理停止。

## 9. 电脑WSL编译（不接硬件）

本课可直接用已核对的Humble编译，不安装/替换训练环境。打开干净WSL终端：

```bash
# 使用系统Python生成ROS消息，退出当前终端里已激活的虚拟环境。
if type deactivate >/dev/null 2>&1; then deactivate; fi
source /opt/ros/humble/setup.bash
cd "$HOME/Jixiebi"/ros2_ws
# 仅构建新增两个包，限制并行以兼顾RK的2GB内存；不会打开串口。
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-select jixiebi_interfaces jixiebi_hardware --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
# source让当前终端能找到刚生成的消息和节点，不会启动硬件。
source install/setup.bash
ros2 interface show jixiebi_interfaces/msg/ServoStatesRaw
```

预期Summary: 2 packages finished；interface show打印自定义字段。CMake通过相对布局定位native并静态链接，不复制第二份驱动，也不启动旧CLI。不要只复制ros2_ws就丢掉同级native。

## 10. 首次部署到RK

按[统一RK部署指南](RK_DEPLOYMENT.md)直接克隆或从电脑当前源码生成学习包，取得两个ROS包及同级`native`。所有命令使用读者自己的RK用户名和地址；不依赖作者的本地压缩包。已经有源码时先检查本地修改，不覆盖已有工程。

## 11. RK使用Jazzy构建

下面在**RK的SSH终端**执行，前置条件为已按部署指南取得源码并安装对应系统的ROS 2 Jazzy及构建依赖：

```bash
if type deactivate >/dev/null 2>&1; then deactivate; fi
# RK使用Jazzy，不能照搬电脑Humble路径。
source /opt/ros/jazzy/setup.bash
cd "$HOME/Jixiebi/ros2_ws"
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-select jixiebi_interfaces jixiebi_hardware --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

预期2 packages finished。若提示缺少ament_cmake/rclcpp/rosidl_default_generators等，停在此处贴完整错误，按缺失项补依赖，不重新安装整个ROS或解锁所有系统包。若新包同名文件已存在，不覆盖用户修改；此前已完整部署则跳过解包。

## 12. RK第一个SSH窗口启动节点

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/Jixiebi/ros2_ws/install/setup.bash"
if fuser -s /dev/ttyUSB0; then
  fuser -v /dev/ttyUSB0
  echo '先在对应窗口Ctrl+C退出旧串口程序'
else
  # 初始化ROS→创建节点和reader→spin定时查询/发布。保持窗口打开。
  ros2 run jixiebi_hardware servo_state_publisher --ros-args \
    -p port:=/dev/ttyUSB0 -p period_ms:=1000 -p query_timeout_ms:=500
fi
```

日志先显示只读节点就绪，后续应出现发布批次、有效6/6、complete=true，以及底层TX/RX。节点持续占有串口，不能同时运行第五课CLI或旧raw_state节点。本课不需要ARM_PROJECT_ROOT或PYTHONPATH指向Python项目。

## 13. RK第二个SSH窗口订阅

在电脑另开Windows PowerShell，用部署指南中的`ssh "${rkUser}@${rkHost}"`登录同一个RK。变量需在该新窗口重新设置。以下在新RK窗口执行，每个新终端都要source：

```bash
if type deactivate >/dev/null 2>&1; then deactivate; fi
source /opt/ros/jazzy/setup.bash
source "$HOME/Jixiebi/ros2_ws/install/setup.bash"
# 看发布节点是否存在；预期包含/jixiebi_servo_state_cpp。
ros2 node list
# 查话题类型，预期jixiebi_interfaces/msg/ServoStatesRaw。
ros2 topic type /jixiebi/servo_states_cpp
# 只订阅一条当前数据，最多等10秒；订阅者不打开串口。
ros2 topic echo /jixiebi/servo_states_cpp --once --full-length --timeout 10 \
  --qos-reliability reliable --qos-durability volatile
```

预期source: SERIAL_TTY、simultaneous: false、complete: true、valid_count: 6，servos里六项servo_id为0～5、status为1、position_p为现场P。成功后可再用ros2 interface show查看状态常量；无需重复运行硬件CLI。

若没有新数据，先看第一窗口是否报错暂停；单有发布者不代表仍有数据。若提示Invalid message type，确认第二窗口也source了本工作区install。若QoS有问题，用ros2 topic info -v查看，保持可靠/volatile匹配，不随意改为读旧缓存。完成后在第一窗口Ctrl+C释放串口。

## 14. 实际验证范围

作者在WSL Humble构建两包并通过4项ROS＋PTY集成检查：真实rclpy接收C++自定义消息、连续批次、逐轴字节/分片、失败批次无旧P复用并暂停持锁、非法/缺省参数不碰TTY。公开仓库的ROS CI使用伪终端重现消息收发；这些检查与下述作者实物记录分别报告，不依赖作者本机日志。

开发者集成测试须先source ROS与install，再用/usr/bin/python3 native/tests/test_ros_cpp_state_publisher.py；它只访问测试自建PTY及本机话题，默认离线仓库测试不启动本节点或访问设备。

作者2026-11-17记录的RK Jazzy节点/话题结果：节点/jixiebi_servo_state_cpp存在，类型jixiebi_interfaces/msg/ServoStatesRaw正确，收到sequence=39、source=SERIAL_TTY、complete=true、valid_count=6；六项status均1、error空，P为1511/1505/1492/1497/1503/1480，整批43.06305ms。该记录只说明作者该次RK本机ROS状态发布/订阅通过；原始硬件日志不随公开仓库分发。读者仍需保留自己的构建与实物结果。

### 为什么两个窗口的输出不同、为什么会一直循环

“第一RK/第二RK”在本课是同一块RK的两个SSH窗口。第一窗口运行C++发布进程，第二窗口运行ros2 topic echo订阅进程；电脑只是显示SSH终端，尚未验收PC↔RK跨机DDS通信。

timer在一次查询完成后等待1000ms再查询/发布，因此看到多轮000～005的TX/RX是预期行为。此次查询约43ms，附近周期可理解为约1043ms，不把单条样本推成长期稳定频率。sequence=39是节点的第39批序号，不是订阅端已收到39条，也不是舵机原生请求编号。

echo --once只让第二窗口收到一条后退出，第一窗口的spin/定时器仍运行。作者第一窗口的早期001=1506和第二窗口第39批001=1505来自不同批次，不需要逐字一致；不能仅凭1P差值归因某个机械误差。simultaneous=false与逐渐增加的finished_offset_ms说明六轴是顺序读取。

需要结束发布时在第一窗口Ctrl+C；只结束echo不会停止发布者。无需为了本次验收再运行旧CLI或重复查询。

本课讲法与ROS官方最小C++发布者相同：Node→Publisher→Timer→publish→spin，具体查询复用已有设备层。[官方最小发布者源码](https://github.com/ros2/examples/blob/jazzy/rclcpp/topics/minimal_publisher/member_function.cpp) · [自定义接口教程源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.rst)
