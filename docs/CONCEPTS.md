# 基础概念与代码阅读

## 数据和状态

- **P 值**：舵机协议的位置数字，不直接等于角度、毫米或夹爪开口宽度；映射需要按设备标定。协议数值边界也不是机械臂实际安全行程。
- **Node**：ROS 2 的程序节点，例如只读状态节点；节点运行不代表传感器数据一定持续更新。
- **Topic**：节点发布/订阅消息的通道，例如 `/jixiebi/servo_states_raw`。本项目当前原始状态使用 JSON 字符串，舵机 P 不能直接填进弧度制关节状态。
- **Action**：ROS 2 中适合长任务的接口机制，通常包含目标、反馈和结果。它是后续可选扩展，不意味着本项目已接入完整 Action 运动执行。
- **TF**：描述坐标系之间相对位置和方向的变换。标称模型计算出的 TF 不等于已实测的物理坐标，也不能独立保证碰撞安全。

## 从协议读到 ROS

1. `src/arm_project/servo_protocol.py` 组装和解析白名单协议帧。例：`#004PRAD!` 是查询 004 号舵机位置的请求；`#004P1540T1500!` 表示协议目标和声明毫秒数，不是运动授权。
2. `src/arm_project/arm_link.py` 管理串口事务、数据解析、超时和记录；回包不包含设备采样时间或请求序号，不应把主机接收时间当作设备采样时间。
3. `src/arm_project/unified_arm.py` 与 `joint_sweep.py` 实现不同的受限控制流程；连续扫掠是独立审阅计划，已有预算和历史运动不能自动重发。
4. `src/arm_project/ros_readonly.py` 与 `ros2_ws/src/jixiebi_driver/jixiebi_driver/raw_state.py` 将只读状态发布到 ROS。
5. `src/arm_project/ros_sweep.py` 与 `ros2_ws/src/jixiebi_driver/jixiebi_driver/single_sweep.py` 对接本地预审的有限单轴动作；需区分请求成功、方向响应和严格目标到位。

## 请求与回复示意

以下摘录是阅读用，不是可单独运行的程序：

```python
# make_query 根据轴号和字段生成只读请求，例如 004 的位置查询。
command = make_query(servo_id, field)

# 通过已有传输对象写串口；字节未完整写出时抛错，不自动重试。
if self.transport.write(command) != len(command):
    raise OSError("query short write; not retried")
```

错误、超时、半帧和无关回复的完整处理请阅读源文件。不要把示例命令直接当作真实机械动作，或把模型输出绕过项目检查直接写入串口。
