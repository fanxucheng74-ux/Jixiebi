# 第十四课 · 在 RViz2 看 RK 的关节反馈

看到底座坐标轴和彩色整臂后，可进入[第十五课：用 C++ 写第一条关节轨迹](CPP_LESSON15_TRAJECTORY.md)。

沿用第十三课接线和网络。RK 的 C++ 发布器读取六轴反馈，WSL 的 C++ 桥把前四轴反馈换成弧度，`robot_state_publisher` 据此计算 TF，RViz2 摆放连杆。先用项目已有的标称映射看模型。需要量化本机角度误差时，再做下面的实测标定。

## 日常使用：复用已有关节方向和比例

10 月 31 日底座下发目标 1550 P 时，实机反馈从 1512 增到 1538 P，其他关节的局部正反向反馈也已留档。旧模型已处理厂家偏航正方向与 URDF 正方向的差别。新的有类型反馈沿用同一换算函数，构建时提取模型系数，无需重做单轴运动。[模型尺寸、方向与验证范围](ARM_FACTS.md)

**RK 的 SSH 终端**沿用第十三课发布器（启动命令见第 2 节）。**电脑 WSL** 构建一次后，每次启动显示只需：

```bash
# 电脑 WSL：加载工作区并接收与 RK 同一域的反馈。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=21 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 看底座坐标轴：本机约定 1511 P 为正前，观察 yaw_link 的红 X 和绿 Y。
ros2 launch jixiebi_joint_state_bridge lesson14_feedback.launch.py mapping:=nominal yaw_zero_p:=1511 view:=axis
```

想看彩色整臂，在 WSL 按 Ctrl+C 关闭上一个视图，再执行：

```bash
ros2 launch jixiebi_joint_state_bridge lesson14_feedback.launch.py mapping:=nominal yaw_zero_p:=1511 view:=model
```

**到这里就完成日常显示。** `view` 选择画面，`mapping` 选择如何把 P 换成角度。看到“尚未启用任何实测关节”或旧版“至少需要一个关节的实测两点标定”，说明命令读入了标定草稿。直接使用上面的显示命令。下面的 1～5 节供需要独立测量本机角度比例时选做。

取 `k = (270/2000) × π/180` rad/P。这台机器四个模型关节依次显示为 `(P0−1511)k`、`π/2+(P1−1500)k`、`−(P2−1500)k`、`(P3−1500)k`。例如底座回读 1521 P 时，显示相对左转 1.35°。004 滚转与 005 夹爪在六轴原始消息中可查；第十二课模型只定义前四个转轴。0.9° 是现场对初始朝向的估读，已按用户选择计入工作零位。

在相同环境的第二个 WSL 终端可运行 `ros2 topic echo /nominal_estimate/joint_states --once`。此处 P 来自 RK；角度按标称映射估计，不能据此判定夹爪离桌高度或决定运动。该 launch 的关节话题、TF、静态 TF 和模型描述都使用 `/nominal_estimate/` 前缀。实测两点模式仍使用 `/joint_states`。

**2026-11-19 实机显示记录。** WSL Humble 收到 RK Jazzy 第 198 批，六轴均有效。000～005 为 1510、1498、1500、1502、1499、1482 P。WSL 同时间戳的四轴输出依次为 −0.135°、89.730°、0°、+0.270°。四条动态 TF 与固定工具端均已收到，RViz2 显示近竖直的蓝色上臂、绿色前臂和橙色腕杆。本轮没有发运动指令，尚未评估运动中的角度误差或延迟。跨版本 DDS 再次打印过 `sequence size exceeds remaining buffer`，来源尚未定位。

**为什么 RViz2 这次能显示。** 跨机用第十三课的热点单播发现，电脑自己的显示节点另外允许回环发现。仅有网格时曾查到 TF 正常、模型外形仍为空；现在 RViz2 直接加载本机从第十二课 Xacro 展开的 URDF，和状态发布器使用同一份模型，姿态随新的 TF 更新。

## 1. 给第一个读数配上正确的角

俯视底座，在**固定底板**上标正前方，在**旋转台或肩支架**上选一条随底座转动的方向线。两线重合为底座 0°。从上往下看，逆时针为正。直立时爪尖可能就在转轴上，所以要看旋转台上的标记。肩关节则从水平前伸量起，上臂向上竖直约为 +90°。

| 舵机 | 第十二课的关节 | 角度依据 |
|---|---|---|
| 000 | `base_yaw` | 俯视，固定底板正前到旋转台方向线的有符号角 |
| 001 | `shoulder_pitch` | 随底座转向的侧视平面内，水平前方到上臂的角 |
| 002 | `elbow_flex` | 上臂向远端的延长线到前臂的相对角，两杆同向共线时为 0° |
| 003 | `wrist_pitch` | 前臂延长线到工具纵轴的相对角，两者同向共线时为 0° |

肩、肘、腕都按各自父连杆局部 +X 的右手方向记正负。可以从 **+X 一侧朝原点**观察，零姿下从 +Y 水平前方向 +Z 抬起是正转。保持视线与转轴平行，改变另一关节时保持父子相对角的定义。004 是末端滚转、005 是夹爪，六轴原始话题均保留；现有 URDF 只有上表四个转动关节。

**本机的第一点 A**：11 月 18 日，WSL 第 1134～1138 批的底座回读均为 1511 P；肩约竖直、1506 P。11 月 19 日现场又观察到底座偏左约 0.9°，用户决定直接用初始朝向作为**工作相对零位**。因此底座以 1511 P 对应 0.0°，后面的转角都从这条初始方向线量起。把底座转到另一处、停稳并同时记录 P 与相对角，就是 B 点。有 A 能定零位，有 A 和 B 才能求每 P 对应的角度。

## 2. 在 RK 发布；需要实测比例时，在 WSL 采 A、B 和检验点 C

**RK 的 SSH 窗口**：若第十三课正在输出“有效 6/6”，让它继续。重新开机时，在 RK 执行下段。它启动已有的只读发布器。

```bash
source /opt/ros/jazzy/setup.bash && source "$HOME/Jixiebi/ros2_ws/install/setup.bash" && {
  unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_DOMAIN_ID=21 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  if fuser -s /dev/ttyUSB0; then fuser -v /dev/ttyUSB0; else
    ros2 run jixiebi_hardware servo_state_publisher --ros-args \
      -p port:=/dev/ttyUSB0 -p period_ms:=1000 -p query_timeout_ms:=500
  fi
}
```

**电脑 WSL，每个新终端先加载环境**。下面最后一行调用采样小工具，按 ID 打印五个新批次、P 中位数及范围。这个辅助脚本用 Python 订阅原始话题；第 4 步的实时转换使用 C++。

```bash
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=21 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 姿态静止、量具已读好时执行。更换关节时把 0 改为相应 ID。
/usr/bin/python3 $HOME/Jixiebi/tools/sample_joint_feedback.py --id 0
```

1. **A**：在纸上画下当前底盘标记的方向线。这台机器可以从刚记录的 1511 P 开始。把纸相对桌子和底板固定。
2. **B**：用你已经熟悉的正常定位或厂家示教方式，让底座在已知可用范围内转到另一处，停稳后再记实际角与五批 P。不要硬扳上电的舵机。两点间的角差应明显大于读角误差，P 差也应明显大于这五次的波动。若眼下只能保持 A，就先完成编译，等能取得 B 再启用该关节。
3. **不用量角器也能取得 B**。可用浏览器按 A4 横向、100% 打印[俯视角度纸](lesson14_yaw_scale.svg)，用直尺核对纸上的 100 mm 校尺。把十字对准旋转轴心，以 A 为相对零线读数。或用直尺在纸上作图：从轴心沿 A 画直线，在正前 100 mm 处作垂线，延长 B 的方向线到这条垂线。量交点到正前线的横向距离 `d`（毫米，左正右负），相对转角 = `atan2(d, 100) × 180/π`。例如横向偏左 17.6 mm，约 +10.0°。这是读角方法，示例不指定实物目标。保持纸面水平，俯视以减小视差；转动时移开尺子。
4. **C**：在 A 和 B 之间独立取第三姿态，用相同方法记录外测角和 P。用 A、B 拟合，在 C 比较“换算角 − 实测角”；记录差几度及量具分辨率。有条件时反方向再回到 C，比较回差。若 C 的偏差超出显示需求，重新检查标记、零位和回差，再决定是否需要分段标定。

| 点 | 外测有符号角（度） | 实读 P 中位数、范围 | 用途 |
|---|---|---|---|
| A | 0.0（以当前标记作相对零位时） | 1511，1511～1511 | 第一端点 |
| B | 现场填写 | 现场填写 | 第二端点 |
| C | 独立读数 | 独立读数 | 检验方向、角误差 |

## 3. 写出 C++ 换算，把弧度填入标准消息

在 **WSL** 打开 `$HOME/Jixiebi/ros2_ws/src/jixiebi_joint_state_bridge/src/calibrated_feedback.cpp`，找到 `convert_feedback()`。它收到一批原始消息，读取本轴实际反馈 `p`。先写出两点插值，再填名字和位置，与文件对照：

```cpp
// 浮点除法算出 P 从 A 走到 B 的比例。分母保留符号，P 反向增长也适用。
const double fraction = static_cast<double>(p - c.p_a) / (c.p_b - c.p_a);
// A 是有角度基准的点；当 P 恰为 A 或 B，两端都应还原外测角。
const double angle_deg = c.angle_a_deg + fraction * (c.angle_b_deg - c.angle_a_deg);
// 源码在这里先追加关节名，再用相同下标写入该关节的弧度。
next.name.emplace_back(kJointNames[id]);
next.position.push_back(angle_deg * kRadPerDegree);
```

拿一个**纯算术例子**验算：若 A 为 1200 P / +20°，B 为 1440 P / −10°，在 1320 P 时比例是 0.5，角度为 +5°，即约 +0.0873 rad。此例的 P 增大而角度减小，不需要再手动翻转符号。每个关节有自己的两点，一组数只在这两个 P 之间有效。

然后打开同目录的 [joint_state_bridge.cpp](../../ros2_ws/src/jixiebi_joint_state_bridge/src/joint_state_bridge.cpp)。沿着这三处读：

- **构造函数**从 YAML 读每轴的 A/B，检查 P、有限角度和关节名。创建 `/joint_states` 的发布者，以及 `/jixiebi/servo_states_cpp` 的订阅者，保留两个对象。启动 `rclcpp::spin` 后，每来一批反馈，ROS 自动调用回调。
- **回调**把收到的批次交给 [convert_feedback](../../ros2_ws/src/jixiebi_joint_state_bridge/src/calibrated_feedback.cpp)。校验来源、六轴的 ID、状态和时间字段，再为已经标定的关节成对填写 `name` 和 `position`。越出实测区间会跳过整批并给出原因。缺少速度或力矩测量时对应数组留空。[JointState 消息定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg)
- **发布**保留 RK 的批次结束时间戳。舵机是逐个读取的（上一课约 42 ms 才读完一批），这里给 RViz 一幅近似姿态，适用于静止或慢速观察。高速动作的同步状态估计需要另行处理。后续 TF 的计算由 `robot_state_publisher` 完成。[状态发布器接口](https://github.com/ros/robot_state_publisher/blob/humble/README.md)

下面是节点构造函数中配对的两个对象。可以顺着最后一行进入 `publish_feedback()`，找到回调里真正发送消息的 `publisher_->publish(output)`。

```cpp
// 输出是标准 JointState，输入仍是第六课定义的 RawBatch。
publisher_ = create_publisher<jixiebi::JointState>("/joint_states", qos);
subscription_ = create_subscription<jixiebi::RawBatch>("/jixiebi/servo_states_cpp", qos,
    [this](jixiebi::RawBatch::ConstSharedPtr raw) { publish_feedback(*raw); });
```

## 4. 在 WSL 编译、填入本机参数

```bash
# 电脑 WSL：在工作区根编译接口、模型包和转换节点。RK 沿用第十三课的程序。
cd $HOME/Jixiebi/ros2_ws
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-up-to jixiebi_joint_state_bridge \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 标定草稿保存在当前用户的配置目录，复制一次即可。
SITE=$HOME/.config/jixiebi/joints.site.yaml
mkdir -p "$(dirname "$SITE")"
test -f "$SITE" || cp src/jixiebi_joint_state_bridge/config/calibration.template.yaml "$SITE"
nano "$SITE"
```

本机草稿中 A 已作注释。**拿到 B 后**，在 `base_yaw` 下把 `servo_id` 改为整数 `0`，取消四个端点字段的注释，填入 `p_a`、`angle_a_deg`、`p_b`、`angle_b_deg`。P 用整数，角用小数（例如 `0.0`）。保存后重启转换节点，参数文件无需重新编译。还没有两点的关节保留 `servo_id: -1`，端点保持注释。代码校验的 2 P 最小跨度只为保证有可插值的整数点；测量精度要用 C 点判别。[参数模板](../../ros2_ws/src/jixiebi_joint_state_bridge/config/calibration.template.yaml)

## 5. 选做：用已填好 A、B 的实测文件检查显示

```bash
# 电脑 WSL：沿用第2步的域21环境；新终端也要 source 两个 setup.bash。
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=21 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
SITE=$HOME/.config/jixiebi/joints.site.yaml
ros2 launch jixiebi_joint_state_bridge lesson14_feedback.launch.py mapping:=measured calibration_file:="$SITE" view:=axis
```

启动文件在 [launch/lesson14_feedback.launch.py](../../ros2_ws/src/jixiebi_joint_state_bridge/launch/lesson14_feedback.launch.py)。它打开桥、状态发布器和 RViz2，不启动第十二课的滑块。模型的 `yaw_link` 本身没有几何外形；在单轴视图里看**离底座 0.170 m 的红 X、绿 Y 两根轴**。实际底座相对 A 逆时针转，它们应随底座向同一方向转。固定坐标系为 `base_link`，蓝 Z 的方向保持朝上。

```bash
# 电脑另一个 WSL 窗口：加载工作区，核对相同姿态的弧度及 TF。
source /opt/ros/humble/setup.bash && source $HOME/Jixiebi/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=21 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
timeout 12s ros2 topic echo /joint_states sensor_msgs/msg/JointState --once
ros2 run tf2_ros tf2_echo base_link yaw_link   # Ctrl+C 结束；看 yaw 角随底座变化
```

给 001、002、003 分别完成两点标定后，在同一 YAML 中启用这三轴。关闭上一个 launch，在其 WSL 窗口把最后的 `view:=axis` 换成 `view:=model` 重启，蓝色上臂、绿色前臂、橙色腕杆会使用各自的反馈。缺少父关节变换时，底座到该子连杆的链尚不完整。RViz 停在旧姿态时，先看 RK 的新批次及桥的提示，再看 `/joint_states` 是否还有新时间戳；避免在域 21 同时开启第十二课的滑块。结束时 WSL 的 launch 和 RK 的发布器分别按 Ctrl+C。

**本课验收范围**：WSL 已从真机采集底座 A 点的五批 P。用合成标定和合成反馈验证了 ROS 2 → JointState → TF → RViz2，单轴视图中的底座坐标轴从 0° 转到 +45°；四轴模型显示 [45°, 15°, −60°, 15°] 时，末端坐标与独立正解一致。四轴合成试验在仅有回环的隔离测试网络里通过，普通镜像网络的本机四轴试验曾超时。现场 B、C 以及实际关节转动与显示的对应关系仍待记录。
