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

[统一RK部署指南](RK_DEPLOYMENT.md)提供从公开源码开始的首次部署、只读命令和故障处理。部署包由当前检出的源码生成，既不依赖作者私有文件，也不包含实验日志或动作预算。

每课源码已提供中文注释。建议依次打开头文件、核心函数、`main`，先预测修改的输出，再编译；不要把多个演示程序的`main`复制进同一个文件。公开默认检查不打开物理串口/相机；PTY仅是本地伪终端模拟，不能代替电气与实物验收。

完成第六课后，后续计划为Service、模拟Action、PC上的RViz Marker、动态参数/Launch、tf2与URDF，再接统一版本的PC↔RK只读联机和有限运动。后续内容未实现时不当成已有功能或可执行命令。
