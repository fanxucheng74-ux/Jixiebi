# 历史实现与对照资料

当前学习主线是仓库根目录的 `native/` C/C++协议和驱动，以及 `ros2_ws/src/jixiebi_interfaces`、`ros2_ws/src/jixiebi_hardware` 两个 ROS 2 包。请从[项目首页](../README.md)进入当前课程。

此目录保留此前方向，便于查看原理、失败处理和接口演进，不代表它们已并入当前C++主线，也不作为新用户的默认启动入口。

| 分类 | 位置 | 保留目的 |
| --- | --- | --- |
| 旧Python ROS驱动与任务适配 | [jixiebi_driver](ros2_ws/src/jixiebi_driver/package.xml) | 对照状态发布、规划预览和旧任务接口；离线测试继续覆盖 |
| 旧标称URDF | [arm_nominal.urdf](ros2_ws/src/jixiebi_description/urdf/arm_nominal.urdf) | 模型与TF学习记录；不当作实物尺寸或空间精度标定 |
| 旧Python静态图视觉节点 | [static_image.py](ros2_ws/src/jixiebi_vision/jixiebi_vision/static_image.py) | HSV颜色候选示例，不代表实时相机或视觉抓放已完成 |
| ESP32/MicroPython尝试 | [wifi_link.py](firmware/esp32/wifi_link.py) | 模拟连接流程与早期方向对照；当前方案不使用ESP32 |
| Python协议、规划与视觉基线 | 根目录 [src/arm_project](../src/arm_project/__init__.py) | 为包导入、CLI、已有测试和参考分析保留原路径，未物理迁入legacy |

保留Python原路径是兼容选择：公开版已具有合成夹具、跨平台测试和默认离线入口，不能为分类而破坏它们。当前学习不从旧运动CLI或旧ROS节点开始。历史代码中的实验结论只适用于对应记录，不能直接用旧计划控制新装配的机械臂。

常规公开检查执行根目录的 `tools/verify_project.py`；默认不要求私有实验日志。只有作者另备完整私有资料时才使用 `--private-evidence`，缺资料会报告失败，不会把未执行的审计算成通过。所有默认检查都不打开串口或摄像头。
