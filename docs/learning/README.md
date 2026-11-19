# C/C++与ROS 2学习主线

先在电脑WSL完成不接设备的C示例，再在RK读取一个舵机、读取一批六轴，最后接入ROS 2话题。当前学习代码位于仓库根目录的`native/`与`ros2_ws/src/`；历史Python、旧ROS和ESP32代码见[历史代码索引](../../legacy/README.md)。

| 顺序 | 本课重点 | 执行位置 |
| --- | --- | --- |
| [第一课：查询组帧](C_LESSON01_QUERY.md) | 数组、容量、格式化、返回码 | 电脑WSL，无硬件 |
| [第二课：完整回包解析](C_LESSON02_REPLY.md) | 长度、帧标记、数字、编号校验 | 电脑WSL，无硬件 |
| [第三课：半包和粘包](C_LESSON03_RECEIVER.md) | 结构体、跨次接收状态、边界与重同步 | 电脑WSL，无硬件 |
| [第四课：C++串口查询](CPP_LESSON04_SERIAL.md) | RAII、Linux TTY、写入、有限等待和解析 | WSL构建；RK只读 |
| [第五课：六轴结构化查询](CPP_LESSON05_ARM_STATE.md) | 引用、固定数组、顺序查询、部分失败 | WSL构建；RK只读 |
| [第六课：rclcpp状态话题](CPP_LESSON06_ROS_PUBLISHER.md) | Node、Publisher、Timer、消息与spin | WSL构建；RK本机收发 |
| [7：Service请求与响应](CPP_LESSON07_SERVICE.md) | 自定义srv、服务回调、客户端、future与有界等待 | PC WSL，静态软件示例 |
| [8：Action模拟任务](CPP_LESSON08_ACTION.md) | 发送目标、订阅进度、取消并读取最终结果 | PC WSL，两个本机C++节点 |
| [9：在RViz2显示方块](CPP_LESSON09_MARKER.md) | 发布标准Marker，理解坐标系、中心、边长与颜色 | PC WSL，C++节点与RViz2图形窗口 |
| [10：在线改颜色与尺寸](CPP_LESSON10_PARAMETERS.md) | 声明和校验节点参数，用 Launch 同时启动 C++ 节点与 RViz2 | PC WSL，一边观察一边调参 |
| [11：从底座看工具的位置](CPP_LESSON11_TF.md) | 用 C++ 广播静态和动态 TF，用 tf2_echo 查父子及跨级变换，在 RViz2 看坐标轴 | PC WSL，模拟工具绕底座转动 |
| [12：让机械臂模型动起来](LESSON12_URDF_XACRO.md) | 写 Xacro 连杆和关节，用模拟 JointState 驱动 robot_state_publisher，在 RViz2 看整条臂 | PC WSL，四个关节滑块 |
| [13：从 WSL 接收 RK 六轴反馈](LESSON13_RK_TO_WSL.md) | 局域网发现、同域订阅、按批次区分网络与设备查询问题 | RK Jazzy 发布，PC WSL Humble 订阅 |
| [14：从实读 P 到 RViz2 关节角](LESSON14_FEEDBACK_JOINT_STATES.md) | 复用厂家方向与工作零位显示四轴；按需要用两点标定和第三点检验实际角度 | RK 沿用第十三课，转换和显示在电脑 WSL |
| [15：写第一条关节轨迹](CPP_LESSON15_TRAJECTORY.md) | 用 C++ 把第 198 批的四轴姿态写成标准 JointTrajectory，在话题中核对列名、弧度、累计到达时间 | 电脑 WSL，发布与订阅；RK 可关机 |
| [16：让 RViz2 播放这条轨迹](CPP_LESSON16_TRAJECTORY_PLAYBACK.md) | 用单调时钟找出当前路段，以三次插值生成四轴 JointState，看肘在第 2 秒换向、第 4 秒返回 | 电脑 WSL 两个终端，复用第十二课彩色模型 |
| [17：通过 Action 播放与取消](CPP_LESSON17_FOLLOW_ACTION.md) | 用标准 FollowJointTrajectory 发送同一条轨迹，读取 desired/actual，核对成功、取消和姿态保持 | 电脑 WSL 两个终端，RViz2 模型 |
| [18：从当前姿态接着走](CPP_LESSON18_FROM_STATE.md) | 读取模型 JointState 生成两点轨迹，核对起点，从取消位置平滑走向指定肘角 | 电脑 WSL 两个终端，沿用轨迹 Action 与彩色模型 |
| [19：从关节角算工具端位置](CPP_LESSON19_TOOL_POSITION.md) | 读取运行中的 URDF 和四轴 JointState，用 KDL 正运动学计算黄色工具点相对底座的 XYZ | 电脑 WSL，在两个静止模型姿态比较毫米坐标 |
| [20：从工具坐标反求关节角](CPP_LESSON20_TOOL_IK.md) | 输入 XYZ 与工具俯仰，求腕点、选肘角分支，用模型 Action 播放并回读工具点 | 电脑 WSL，沿用第十九课模型和两个终端 |
| [21：让工具端走直线](CPP_LESSON21_CARTESIAN_LINE.md) | 在线段上取点并逐点逆解，用 Action 反馈画出球心轨迹，比较沿途和终点的偏差 | 电脑 WSL，RViz2 双终端实验 |
| [22：给直线轨迹安排巡航速度](CPP_LESSON22_SPEED_PROFILE.md) | 梯形速度时间律、逐点逆解、从相邻模型反馈求速率，比较 35 和 25 mm/s | 电脑 WSL，RViz2 看轨迹、终端测速 |
| [23：比较起停时的加速度](CPP_LESSON23_ACCELERATION.md) | 同一条线、同一用时，比较梯形速度与三次平滑速度，从三个球心位置估算加速度 | 电脑 WSL，RViz2 和速度、加速度对照图 |

[统一RK部署指南](RK_DEPLOYMENT.md)提供从公开源码开始的首次部署、只读命令和故障处理。部署包由当前检出的源码生成，既不依赖作者私有文件，也不包含实验日志或动作预算。

每课源码已提供中文注释。建议依次打开头文件、核心函数、`main`，先预测修改的输出，再编译；不要把多个演示程序的`main`复制进同一个文件。公开默认检查不打开物理串口/相机；PTY仅是本地伪终端模拟，不能代替电气与实物验收。

## WSL 课程环境

文中使用 Ubuntu 22.04、ROS 2 Humble 桌面版和 WSLg，示例克隆目录为 `$HOME/Jixiebi`。安装桌面版与 rosdep 后，在新终端执行：

```bash
cd "$HOME/Jixiebi"
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y
sudo apt-get install python3-colcon-common-extensions python3-matplotlib
cd ros2_ws
colcon build --packages-up-to jixiebi_learning jixiebi_joint_state_bridge
source install/setup.bash
```

第 7～12 课学习通信与建模，第 13～14 课观察实物只读反馈，第 15～23 课使用本机模型。可直接从[第 23 课的两终端实验](CPP_LESSON23_ACCELERATION.md)比较同一条横移的速度与加速度。表中读数与坐标约定见[课程事实表](ARM_FACTS.md)。
