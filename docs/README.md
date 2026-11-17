# 代码与文档分类

当前入口是[项目首页](../README.md)和[六节C/C++课程](learning/C_LESSON01_QUERY.md)，不是历史Python运动CLI。

| 分类 | 入口 | 说明 |
| --- | --- | --- |
| C/C++主线 | [native](../native/arm_driver/include/arm_state.hpp) | 协议、串口、单轴与六轴只读；代码中的P仍为协议单位 |
| ROS 2 C++主线 | [rclcpp节点](../ros2_ws/src/jixiebi_hardware/src/servo_state_publisher.cpp) | 复用唯一串口所有者，发布类型化原始状态 |
| 当前教程 | [课程1](learning/C_LESSON01_QUERY.md) → [课程6](learning/CPP_LESSON06_ROS_PUBLISHER.md) | 中文讲解与WSL/RK命令 |
| 上板部署 | [统一部署](learning/RK_DEPLOYMENT.md) | 克隆公开源码或本地白名单打包，不依赖作者电脑文件 |
| 项目计划 | [FULL_ARM_PLAN](FULL_ARM_PLAN.md) | PC可视化、RK驱动与后续课程 |
| 当前证据 | [CONTEXT](CONTEXT.md) | 实测、模拟和待验证分开记录 |
| 历史Python环境 | [QUICKSTART](QUICKSTART.md) | 兼容的Python离线基线，不作为新设备层主线 |
| 历史模型/运动/视觉 | [legacy](../legacy/README.md) | 保留学习与回归资料，不自动接入当前C++控制 |
| 历史控制原理 | [FINITE_MOTION](FINITE_MOTION.md)、[CONTROL_FIXES](CONTROL_FIXES.md) | 旧实现边界，不等于新C++动作已完成 |

Python包仍在src/arm_project中，保留安装、CLI和既有测试兼容。旧ROS包和ESP32代码已物理归档到legacy，当前ros2_ws默认只包含两个C++主线包。分类不删除历史实现，也不放宽动作权限。
