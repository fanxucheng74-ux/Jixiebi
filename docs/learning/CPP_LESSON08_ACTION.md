# C++ 第八课 · 用 ROS 2 Action 执行和取消计数任务

这次让客户端下达“数到 8”的目标，边执行边看服务端反馈，并在看到第 3 步后请求取消。两个终端都开在**电脑 WSL Ubuntu 22.04 / ROS 2 Humble**。运行时只改变程序内的计数器，RK 可以保持关机。

第七课查询课程编号，收到一次响应就能退出。这里需要区分两个时刻：服务端**接受目标**时，计数才要开始；客户端拿到**最终结果**时，才知道这一轮怎样结束。

## 先写接口，约定三个方向的数据

在 WSL 的 `$HOME/Jixiebi/ros2_ws/src/jixiebi_interfaces/action/CountSteps.action` 对照下面三个字段。文件中两个 `---` 的位置有意义。

```text
# Goal：客户端交给服务端的目标步数。
uint32 total_steps
---
# Result：目标结束时，服务端报告实际完成了几步。
uint32 completed_steps
---
# Feedback：执行途中，服务端报告当前完成了几步。
uint32 completed_steps
```

文件顺序是 **Goal、Result、Feedback**。实际运行则先发 Goal，途中可以看到多次 Feedback，最后再取得 Result。后两段都用 `completed_steps`，但它们分别属于 `CountSteps::Feedback` 和 `CountSteps::Result`。最终是成功、取消还是中止，由 ROS 随结果返回的状态码表示。[接口定义](../../ros2_ws/src/jixiebi_interfaces/action/CountSteps.action)

## 按一次调用的顺序读两个 C++ 文件

WSL 项目根目录是 `$HOME/Jixiebi`。先看服务端文件里的 `main()`，再看客户端文件里创建和发送目标的位置。

| 要写或阅读的文件（相对于项目根目录） | 跟读问题 |
| --- | --- |
| `ros2_ws/src/jixiebi_learning/src/lesson08_action_server.cpp` | 服务端什么时候接受目标？究竟哪段代码让计数加一？ |
| `ros2_ws/src/jixiebi_learning/src/lesson08_action_client.cpp` | 发出目标以后，客户端在哪里收进度？如何取消同一个目标并取得结果？ |
| `ros2_ws/src/jixiebi_interfaces/CMakeLists.txt` | `rosidl_generate_interfaces` 如何根据 `.action` 生成 C++ 类型？ |
| `ros2_ws/src/jixiebi_learning/CMakeLists.txt` | 哪两个 `add_executable` 把上述 `.cpp` 编译为两个独立程序？ |

**1. 服务端先等待。** 服务端的 `main()` 启动节点，`rclcpp::spin` 调度回调。收到目标时，服务端检查 `total_steps` 是否在 1～20 之间，以及当前是否已有任务。接受后保存这次目标的句柄（goal handle）。可以把句柄理解为“本次调用对应的那个目标”，之后的反馈和结果都通过它发出。被拒绝的目标不会进入计数阶段。[服务端源码](../../ros2_ws/src/jixiebi_learning/src/lesson08_action_server.cpp)

**2. 客户端发目标。** 客户端把 ROS 参数 `steps` 填入 `Goal.total_steps`，再用 `async_send_goal` 把目标送往 `/jixiebi/learning/count_steps`。目标应答告诉客户端服务端是否接受，接受后客户端也获得这次目标的句柄。`cancel_after` 是客户端自己的参数，它决定何时申请取消，因此 `.action` 的 Goal 段里只有 `total_steps`。[客户端源码](../../ros2_ws/src/jixiebi_learning/src/lesson08_action_client.cpp)

**3. 服务端发反馈，客户端看进度。** 服务端的定时器约每 500 ms 得到一次执行机会。执行一步后，它把当前计数放进 Feedback，通过目标句柄的 `publish_feedback` 发出。定时器每次只做一步就返回，`spin` 在两次计数之间仍有机会处理取消。终端 B 每出现一条 `FEEDBACK=...`，对应的是客户端收到一条进度消息。

跟写时先读服务端的 `advance_one_step()`，它把“完成一步”和“让客户端看见这一步”连在一起。以下是同一分支内的四条语句，省略了总步数读取和日志。

```cpp
++completed_steps_;                                  // 本次目标在服务端完成一步。
auto feedback = std::make_shared<CountSteps::Feedback>();
feedback->completed_steps = completed_steps_;        // 把当前计数写入反馈对象。
active_goal_->publish_feedback(feedback);            // 经这个目标的句柄发给客户端。
```

**4. 客户端主动取消，服务端结束任务。** `cancel_after=0` 表示让目标自然完成，服务端调用 `succeed`；若设为 `3`，客户端收到达到阈值的反馈后，用 `async_cancel_goal` 为这个句柄申请一次取消。服务端接受取消请求后，执行逻辑观察到 `is_canceling()`，将已完成步数填入 Result，再调用 `canceled`。客户端继续处理 ROS 事件，等待最终结果。`CANCEL_ACCEPTED` 表示对方接受了取消请求，`RESULT=CANCELED` 才表示这个目标以取消结束。两者之间可能已有一条反馈在传输，实际停止步数以 Result 为准。

可以在客户端的 `run_task()` 中依次定位以下语句。它们来自函数内的不同位置，完整代码在每一次等待后都检查结果。

```cpp
auto admission = client->async_send_goal(goal, options);  // 发出目标。options 中登记了反馈回调。
const auto handle = admission.get();                      // 应答就绪后取句柄，空句柄表示被拒绝。
auto result = client->async_get_result(handle);           // 为这一个目标登记结果等待。
// 下列调用位于循环内部，只有反馈达到阈值、尚未发送取消且结果仍在等待时才执行。
request_cancel(executor, *client, handle);                // 内部向同一目标调用 async_cancel_goal。
const auto finished = result.get();                       // 循环结束、结果就绪后取状态码和载荷。
```

`options.feedback_callback` 中的 `completed = feedback->completed_steps;` 把每次收到的进度写入局部变量。主循环执行 `executor.spin_once(100ms)` 时可以调度这个回调，随后检查 `completed >= cancel_after`。这样发送取消和等待应答都发生在回调返回之后。同一个 `handle` 穿过申请取消和等待结果两条路径，让每次操作都指向最初接受的目标。

## 终端 A · 编译并启动服务端

第一次学习或修改源码后，在**电脑 WSL 终端 A** 执行。两包分别负责编译接口与节点，`source install/setup.bash` 让本终端找到新程序。

```bash
# 切回系统 Python 并加载 Humble。
if type deactivate >/dev/null 2>&1; then deactivate; fi
source /opt/ros/humble/setup.bash
cd $HOME/Jixiebi/ros2_ws

# 有限并行构建接口包和课程包，成功时末尾显示 2 packages finished。
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-select jixiebi_interfaces jixiebi_learning --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

编译成功后，在同一窗口启动服务端。以后打开新终端可直接执行这一段。

```bash
source /opt/ros/humble/setup.bash
source $HOME/Jixiebi/ros2_ws/install/setup.bash
# 本课只在本机发现节点，两个终端使用同一 ROS 域。
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=17
# 显示目标、结果、反馈三段字段，再启动计数任务服务端。
ros2 interface show jixiebi_interfaces/action/CountSteps
ros2 run jixiebi_learning lesson08_action_server
```

让终端 A 保持打开。服务端就绪后会等客户端发目标。

## 终端 B · 同一个客户端运行两次

在电脑新开另一个 **WSL 终端 B**，先加载与 A 相同的环境。

```bash
source /opt/ros/humble/setup.bash
source $HOME/Jixiebi/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=17

# 第一次：默认 steps=6、cancel_after=0，让任务自然完成。
ros2 run jixiebi_learning lesson08_action_client
```

观察 B 的顺序：接受目标 `GOAL_ACCEPTED` → 多次反馈 `FEEDBACK=...` → `RESULT=SUCCEEDED`、`COMPLETED_STEPS=6`。观察 A，服务端处理完这个目标后继续等待。待 B 回到命令提示符，再执行第二次。

```bash
# 第二次：请求8步，收到至少3步的反馈时，客户端主动申请取消这个目标。
ros2 run jixiebi_learning lesson08_action_client --ros-args \
  -p steps:=8 -p cancel_after:=3
```

留意 B 中的 `CANCEL_REQUESTED`、取消应答和最后的 `RESULT=CANCELED`、`COMPLETED_STEPS=...`。将“触发取消时看到的反馈步数”和“最终完成步数”分别读出。如果没有发现服务端，先看 A 是否仍在运行，并核对两个窗口的 `ROS_DOMAIN_ID` 与 `ROS_LOCALHOST_ONLY` 设置。`Ctrl+C` 在本课用于两次演示结束后退出 A 中的服务端；第二次的取消由 B 的客户端代码发起。

## 动手改一个值

把终端 B 的目标保留为 8，把 `cancel_after:=3` 改成 `cancel_after:=5`。先预测 `CANCEL_REQUESTED` 将出现在第几步反馈之后，再运行验证。修改命令行参数即可看到变化。能在源码中分别找到“发送目标”“发布进度”“请求取消”“取得终态”四处调用后，就可以把同一流程用于理解后面的模拟轨迹任务。
