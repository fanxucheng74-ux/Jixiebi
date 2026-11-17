# C++第五课：结构化结果与六轴顺序查询

## 1. 从已通过的第四课出发

作者2026-11-17在RK实际执行第四课程序，获得TX #002PRAD!、RX #002P1492!、SOURCE=SERIAL_TTY、SERVO_ID=2、POSITION_P=1492。这证明该次002查询的原生C++链路已打通；不是六轴/运动/坐标精度验收，也不把1492作为未来固定位置。

main调用的是完整查询事务，不只是接收函数：组帧→发送→等待→流分帧→内容校验→返回P。第五课不改这条链路，只把它按编号调用六次，保存结果供后续ROS发布使用。学习重点为struct、std::array、引用&和范围for；命令行解析/打印格式先作为参考。

## 2. 文件与调用位置

| 文件 | 新增作用 |
| --- | --- |
| [arm_state.hpp](../../native/arm_driver/include/arm_state.hpp) | 一个样本和一批样本的数据类型、批量函数声明 |
| [arm_state.cpp](../../native/arm_driver/src/arm_state.cpp) | 初始化编号、顺序查询、计时、失败处理 |
| [read_arm_state.cpp](../../native/apps/read_arm_state.cpp) | 新main：一个reader、一次批量调用、打印所有状态 |
| [build_arm_state.sh](../../tools/build_arm_state.sh) | 复用第四课构建，再生成第五课程序 |
| [test_cpp_arm_state.py](../../native/tests/test_cpp_arm_state.py) | 仅连接自建PTY的集成检查，不随实物部署 |

电脑WSL在~/Jixiebi编辑。四个新源文件/脚本已提供参考实现，按同名文件阅读或修改，不在旧main尾部再追加一个main。两个程序各有自己的main，编译脚本分别生成read_position与read_arm_state，不把两个main链接进同一个程序。

## 3. 一个P值为什么不够

第四课成功返回002的1492P，调用者还知道自己正在查哪个编号；六个结果若只用六个裸整数，就容易混淆编号、失败初值和先后时刻。现在用结构体将相关信息放在一起：

```cpp
struct ServoPositionSample {
    std::uint8_t servo_id = 0U;
    SampleStatus status = SampleStatus::NotQueried;
    std::uint16_t position_p = 0U;
    double elapsed_ms = 0.0;
    double finished_offset_ms = 0.0;
    std::string error;
};
```

上面代码中的字段与实际头文件一致，中文单位/边界注释在源文件中。servo_id是编号；status是本行状态；position_p只有成功时有效；elapsed_ms是这一轴查询的主机调用耗时；finished_offset_ms是本次调用完成距批次开始的毫秒偏移；error保存失败原因。

初始化0P不是实际位置。主程序只有status==Ok才打印position_p，失败/未查均打印P=--。

```cpp
enum class SampleStatus {
    NotQueried,
    Ok,
    Failed
};
```

enum class定义一组命名状态，通过SampleStatus::Ok等名字使用。三个状态区分“尚未发查询”“收到合法匹配回复”“已经尝试但失败”，不能只靠P值判断。

## 4. std::array保存一批结果

```cpp
struct ArmStateBatch {
    std::array<ServoPositionSample, kServoCount> samples{};
    std::size_t valid_count = 0U;
    bool complete = false;
    double elapsed_ms = 0.0;
};
```

std::array<类型, 数量>是固定长度数组，支持下标与size()。kServoCount来自SERVO_ID_MAX+1，即0～5共6个地址，不代表已标定六个转动自由度。samples[0]保存000，samples[2]保存002，samples[5]保存005；valid_count是合法结果数，complete只有全部成功才为true。

这里的elapsed_ms是整批耗时，与每行自己的elapsed_ms处于不同对象中。例如batch.elapsed_ms与sample.elapsed_ms不会混在一起。

## 5. main传的是同一个串口对象

下面两行已在新main中，位置是native/apps/read_arm_state.cpp第79～82行：

```cpp
// 构造一次，串口锁维持到本批查询和打印结束。
jixiebi::PositionReader reader(port, pty_simulation);
// 传reader本身的引用，返回完整ArmStateBatch，不再只有一个uint16_t。
const auto batch = jixiebi::read_arm_state(
    reader, std::chrono::milliseconds(timeout_ms));
```

新函数声明为：

```cpp
ArmStateBatch read_arm_state(PositionReader &reader,
                            std::chrono::milliseconds timeout);
```

PositionReader &reader中的&表示引用：函数内部操作main已经创建的那个对象，不复制它，也不打开第二个串口。PositionReader已经禁止复制，因此去掉&按值传递还会编译失败。main中的auto推导为ArmStateBatch；batch.samples是本批六行结果。

## 6. 顺序循环怎么复用旧函数

arm_state.cpp先给六个位置登记编号，再查询。登记与查询分开是为了让中途失败后那些未查询行仍显示正确编号：

```cpp
for (std::size_t index = 0U; index < batch.samples.size(); ++index) {
    batch.samples[index].servo_id = static_cast<std::uint8_t>(index);
}
```

接着的核心是（省略计时，完整实现在同名源文件）：

```cpp
for (auto &sample : batch.samples) {
    try {
        sample.position_p = reader.read_position(sample.servo_id, timeout);
        sample.status = SampleStatus::Ok;
        ++batch.valid_count;
    } catch (const std::exception &error) {
        sample.status = SampleStatus::Failed;
        sample.error = error.what();
    }
    // 实际代码在这里记录本轴耗时与主机完成偏移。
    if (sample.status == SampleStatus::Failed) {
        return batch;
    }
}
batch.complete = true;
return batch;
```

for(auto &sample : batch.samples)依次让sample引用数组里的每一项。&让修改直接保存到数组；若写auto sample得到副本，对副本的修改不会保存回samples。打印时使用const auto &sample，只读、不复制。

读第一轴前，reader就已经打开，整个循环结束后才关闭。不要把构造reader放到循环里，也不启动六个线程抢同一个串口。每个编号只调用一次原read_position，没有失败补发。首个失败即返回此前记录，其他保持NotQueried；主程序显示INCOMPLETE并以非0退出。串口无法打开时直接报ERROR，不伪造六行已查询数据。

每轴timeout默认500ms；最多6次查询，不把它写成六轴必须固定休眠3秒或硬实时保证。FINISHED_OFFSET_MS只是主机函数结束时刻的相对偏移，包含解析/诊断开销；不是舵机采样时间戳。SIMULTANEOUS=false说明这是先后读取，不是同步快照。

## 7. 电脑WSL编译

```bash
cd "$HOME/Jixiebi"
# 阅读新增数据结构，再读核心循环，最后看新main中的调用。
nano native/arm_driver/include/arm_state.hpp
```

```bash
nano native/arm_driver/src/arm_state.cpp
```

```bash
# 只构建两个原生程序，不连设备；最后一行为第五课BUILD_OK。
bash tools/build_arm_state.sh
# 不给端口，帮助不连接串口。
./build/cpp_lesson05/read_arm_state --help
```

## 8. 从仓库源码部署

首次使用RK时，按[统一RK部署指南](RK_DEPLOYMENT.md)直接克隆或从电脑生成学习包。学习包会从当前源码生成，包含本课文件；不需要下载作者的旧增量包。若已部署第四课，更新或合并本课源码时保留自己的修改。

## 9. RK编译

在**RK的SSH终端**、源码根目录执行：

```bash
# 重新编译源码；脚本同时生成第四课和第五课程序，不访问串口。
cd "$HOME/Jixiebi" &&
bash tools/build_arm_state.sh
```

末尾应显示`BUILD_OK: build/cpp_lesson05/read_arm_state`。已有完整源码时不必重传或解包；编译错误先处理，再执行查询。

## 10. RK只读一批六轴

保持此前已读通的ZLink与舵机电源连接，先退出其他串口所有者，在RK执行：

```bash
cd "$HOME/Jixiebi" &&
if fuser -s /dev/ttyUSB0; then
  fuser -v /dev/ttyUSB0
  echo '先在对应窗口Ctrl+C退出旧串口程序，再查询'
else
  # 只有--port，没有--id，因为程序已经规定000～005依次查询各一次。
  ./build/cpp_lesson05/read_arm_state --port /dev/ttyUSB0
fi
```

正常会看到六条TX/匹配RX诊断及六行ID=000～005 STATUS=OK P=... ELAPSED_MS=... FINISHED_OFFSET_MS=...，最后BATCH_STATUS=OK、VALID_COUNT=6、BATCH_ELAPSED_MS=...。失败时保留之前OK行，失败行及未查行P=--，最后INCOMPLETE，不能冒充完整状态。P仍是协议单位。

日常直接在已有目录执行本查询，不必重传/解包。源码变化后重新构建，不自动重试查询。保留完整输出并登记你自己的六轴实机结果；本节不发送P/T运动目标。

### 命令怎样进入查询函数

```bash
./build/cpp_lesson05/read_arm_state --port /dev/ttyUSB0
```

这条是Bash命令：运行已经编译的可执行程序，并传入参数。./从当前目录找起，build/cpp_lesson05/read_arm_state是程序路径，--port和/dev/ttyUSB0是两个参数。它不是直接在终端里调用C++函数，也不是现场编译源码。

新进程进入native/apps/read_arm_state.cpp的main(int argc,char *argv[])，这次argc=3：argv[0]是程序路径，argv[1]为--port，argv[2]为/dev/ttyUSB0。main解析后得到port字符串，创建PositionReader，再执行jixiebi::read_arm_state(reader,timeout)。该函数在arm_state.cpp中循环调用reader.read_position，最后把批次返回给main打印并关闭设备。

命令末尾下一行的fi属于外层Bash结构if...then...else...fi，只负责结束条件判断，不属于查询程序的参数或C++函数。如果fuser发现占用，执行then部分；没有发现占用才执行else部分里的程序。程序正常运行一次只查一批六轴，运行完返回终端；SIMULTANEOUS=false仍表示非同步采样。

**作者2026-11-17实机记录：** 第五课六轴均OK，P依次为1509/1503/1493/1497/1503/1480，VALID_COUNT=6，整批41.297ms。这是作者在特定装配下的一次RK串口批次结果，不是长期频率、设备同步时间或运动/空间精度验收。

## 11. 开发者检查与练习

WSL GCC/G++11.4严格构建与7项PTY检查通过：顺序字节、分片/错ID/回显、首/中轴超时保留部分结果且无重发、整批锁、断线及CLI。开发者命令为python3 native/tests/test_cpp_arm_state.py --binary build/cpp_lesson05/read_arm_state --single-binary build/cpp_lesson04/read_position；检查仅使用新建PTY，不用实物。该结论不代替本课RK实机输出。

练习先用自己的话解释：①samples[2]对应哪个ID；②auto &sample中的&去掉会影响什么；③若002失败，哪些行是OK、FAILED与NOT_QUERIED，能否声明完整状态。继续[C++第六课：rclcpp状态发布](CPP_LESSON06_ROS_PUBLISHER.md)，复用同一批次，不另写一套串口。
