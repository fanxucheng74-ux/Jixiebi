# 代码与文档分类

从[项目首页](../README.md)或[第 1～23 课学习目录](learning/README.md)进入。

| 分类 | 入口 | 说明 |
| --- | --- | --- |
| C/C++主线 | [native](../native/arm_driver/include/arm_state.hpp) | 协议、串口、单轴与六轴只读；代码中的P仍为协议单位 |
| ROS 2 C++主线 | [rclcpp节点](../ros2_ws/src/jixiebi_hardware/src/servo_state_publisher.cpp) | 复用唯一串口所有者，发布类型化原始状态 |
| 当前教程 | [C 协议](learning/C_LESSON01_QUERY.md) → [ROS 通信](learning/CPP_LESSON07_SERVICE.md) → [模型轨迹与测速](learning/CPP_LESSON23_ACCELERATION.md) | 中文讲解与 WSL/RK 命令 |
| 上板部署 | [统一部署](learning/RK_DEPLOYMENT.md) | 克隆公开源码或本地白名单打包，不依赖作者电脑文件 |
| 项目计划 | [FULL_ARM_PLAN](FULL_ARM_PLAN.md) | PC可视化、RK驱动与后续课程 |
| 当前证据 | [CONTEXT](CONTEXT.md) | 实测、模拟和待验证分开记录 |
| 历史Python环境 | [QUICKSTART](QUICKSTART.md) | 兼容的Python离线基线，不作为新设备层主线 |
| 历史模型/运动/视觉 | [legacy](../legacy/README.md) | 保留学习与回归资料，不自动接入当前C++控制 |
| 历史控制原理 | [FINITE_MOTION](FINITE_MOTION.md)、[CONTROL_FIXES](CONTROL_FIXES.md) | 旧实现边界，不等于新C++动作已完成 |

Python 基线在 src/arm_project 中，旧 ROS 包和 ESP32 代码在 legacy 中。当前 ros2_ws 包含协议接口、RK 硬件适配、PC 学习程序和关节反馈桥四个包。
