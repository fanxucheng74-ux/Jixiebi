# 第十三课 · RK 发布六轴反馈，电脑 WSL 收一批

第六课两个 SSH 窗口都在 RK 上运行程序。这次保留 RK 上的 C++ 发布器，把订阅者移到电脑自己的 WSL 终端。目标是：在 **WSL** 打印六个真实舵机的原始位置，用批次号与 **RK** 日志对应。SSH 负责远程启动程序，ROS 2 通过局域网传消息。

```text
舵机 → ZLink → RK 的 /dev/ttyUSB0 → C++ 六轴查询 → ROS 2 / DDS → 电脑 WSL 的 ros2 topic echo
```

## 是否先统一 ROS 2 版本

本课可以先用已有的 RK Ubuntu 24.04 / Jazzy 和 PC WSL Ubuntu 22.04 / Humble 做一次只读互通实验。两端使用同样的六轴消息定义、域 21、Fast DDS 和匹配的 QoS。ROS 官方说明跨发行版通信可能可用，但不提供兼容保证。一次成功可以验证这套配置的这条话题。以后持续做多机开发，建议在 PC 另建 Ubuntu 24.04 / Jazzy 环境，保留已有的 Humble 学习工作区；在新环境从源码另行构建自己的接口包。[ROS 官方跨发行版说明](https://docs.ros.org/en/jazzy/Releases.html#cross-distribution-communications)

## 终端 A · 在 RK 开始只读查询

让电脑和 RK 连上同一局域网，RK、ZLink、舵机保持第六课接线和独立供电。在 Windows PowerShell 输入自己的 SSH 用户和主机地址。

```powershell
# 电脑 PowerShell：建立 SSH 会话。登录后这个窗口里的程序都运行在 RK。
$rkTarget = Read-Host '请输入 RK 的 用户名@主机地址'
ssh $rkTarget
```

在 **RK 的 SSH 窗口** 执行。两条 `source` 都成功后再启动节点：

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/Jixiebi/ros2_ws/install/setup.bash" && {
  # 清除前面本机课程的限制，允许 Jazzy 在本子网发现节点。
  unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  # 域号决定哪些参与者可以相互发现；RMW 选择两端都使用的 Fast DDS。
  export ROS_DOMAIN_ID=21 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  # 查当前用户可见的占用者，避免与第六课节点或串口 CLI 同时打开。
  if fuser -s /dev/ttyUSB0; then
    fuser -v /dev/ttyUSB0
  else
    ros2 run jixiebi_hardware servo_state_publisher --ros-args \
      -p port:=/dev/ttyUSB0 -p period_ms:=1000 -p query_timeout_ms:=500
  fi
}
```

看到“发布批次…，有效6/6，…complete=true”后，保持 A 运行。`period_ms` 是完成一批后到下一批的等待时间，`query_timeout_ms` 是单轴查询的超时时间，单位均为毫秒。端口被占用时，先在原发布节点窗口按 Ctrl+C，再用域 21 启动。如出现“不完整，查询已暂停”，先看 A 的串口错误和舵机供电，按 Ctrl+C 释放串口。

## 终端 B · 在电脑 WSL 直接订阅

打开本机 **Ubuntu-22.04 WSL**。本窗口使用电脑上的 Humble 和已有接口包。

```bash
# 加载本地 ROS 环境、自定义消息；确认此次订阅运行在哪台机器。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
hostname
echo "$ROS_DISTRO"  # 预期 humble

# 与 RK 同域并允许局域网通信。前几课设置的 localhost 限制要在这里清除。
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=21 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 在本机创建订阅者，显式指定自定义类型。等待最多 15 秒，收到一批就退出。
timeout 15s ros2 topic echo /jixiebi/servo_states_cpp \
  jixiebi_interfaces/msg/ServoStatesRaw --once --full-length \
  --qos-reliability reliable --qos-durability volatile --qos-history keep_last --qos-depth 1
```

这里 `reliable` 对应发布端的可靠传输，`volatile` 表示订阅后等新批次，`keep_last` 配合深度 1 只保留最新一条。请在 B 对照 `sequence` 与 A 中同号批次，再检查 `source: SERIAL_TTY`、`complete: true`、`valid_count: 6` 和 ID 0～5 各项 `status: 1`。`simultaneous: false` 表示顺序查询，`position_p` 是舵机协议位置值，`elapsed_ms` 是 RK 查询耗时。B 的订阅退出后 A 会继续查询。全部结束时在 **A 按 Ctrl+C**。

**本次实测**：WSL 按上述默认发现方式收到了第 233 批，六轴有效，ID 0～5 的位置依次为 1511、1506、1502、1502、1504、1480 P，板端读完这批用时 42.026 ms。订阅时还出现过 `sequence size exceeds remaining buffer`，原因待定位。这个结果验证了当前两套发行版之间的这一种单向消息。

## 从实际源码看两台机器的分工

以下语句分别来自 RK 节点的构造函数和定时回调；WSL 端的 `ros2 topic echo` 已经是一个订阅程序。

```cpp
// RK：在构造函数中登记原始六轴消息的发布者。
publisher_ = create_publisher<RawBatch>("/jixiebi/servo_states_cpp", qos);
// RK：在定时回调中取回本批六个位置，随后填写消息。
const auto batch = jixiebi::read_arm_state(
    *reader_, std::chrono::milliseconds(query_timeout_ms_));
message.sequence = ++sequence_;  // 进程内批次号，能与板端日志对照。
publisher_->publish(message);    // WSL 按同一消息定义解出各轴的 position_p。
```

可以分别查看[RK 发布代码](../../ros2_ws/src/jixiebi_hardware/src/servo_state_publisher.cpp)、[整批消息](../../ros2_ws/src/jixiebi_interfaces/msg/ServoStatesRaw.msg)和[每轴字段](../../ros2_ws/src/jixiebi_interfaces/msg/ServoStateRaw.msg)。

## 15 秒没收到时，从哪层查

1. **先看 A。** 若 A 连续显示有效 6/6，保留节点运行，集中检查 PC 到 RK 的 DDS 链路。若 B 报找不到消息包，在 B 用 `ros2 interface show jixiebi_interfaces/msg/ServoStatesRaw` 核对本地接口；本次已确认它能加载，并有六个数组元素。换电脑且缺包时，在 WSL 工作区运行 `colcon build --packages-select jixiebi_interfaces`，随后重新 `source install/setup.bash`。
2. **核对 PC 网络模式。** 在本机 WSL 执行 `wslinfo --networking-mode`。本次切换前为 `nat`，切换后实测为 `mirrored`。SSH 走 TCP，默认 DDS 发现使用局域网 UDP 多播，两条路径可以有不同结果。Windows 11 22H2 及更高版本支持 WSL 镜像网络。[微软 WSL 网络说明](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking)
3. **若选择切换为镜像模式，这是一次性的电脑设置。** 在 Windows PowerShell 运行 `notepad "$env:USERPROFILE\.wslconfig"`，新建的文件要保存为准确的 `.wslconfig` 文件名。在已有的 `[wsl2]` 段修改或添加 `networkingMode=mirrored`；新文件可以写成下面两行。保存所有 WSL 工作并关闭课程窗口，再在 **PowerShell** 运行 `wsl --shutdown`（会停止所有运行中的 WSL 发行版）。重新打开 Ubuntu-22.04，确认 `wslinfo --networking-mode` 输出 `mirrored`，重做终端 B。A 若使用独立 PowerShell SSH，可以继续运行。[微软配置和重启说明](https://learn.microsoft.com/en-us/windows/wsl/wsl-config)

```ini
[wsl2]
networkingMode=mirrored
```

4. **仍然收不到时区分网络和接口。** 在电脑 WSL 运行 `ros2 multicast receive`，在另一条 RK SSH 会话运行 `ros2 multicast send`，测试 RK→WSL 的 UDP 多播；再交换发送和接收端，测反向。接收端按 Ctrl+C 结束。热点的客户端隔离、Windows/Hyper-V 防火墙和板端防火墙都可能影响发现。双向多播通过只证明这项测试可达；再核对两端加载的域、RMW 和两份消息的字段、类型及顺序。正式排查跨版本差异时用同发行版对照。

若 Windows 防火墙阻止 DDS，请按自己的网卡、板端地址与实际 ROS 域配置 UDP 入站规则。域 21 使用的多播端口为 12650/12651；前四个参与者使用的单播端口为 12660～12667。[微软 Hyper-V 防火墙文档](https://learn.microsoft.com/en-us/windows/security/operating-system-security/network-security/windows-firewall/hyper-v-firewall)
