# C++第四课：打开串口，串起查询与回包解析

完成前三课后，本课采用“C协议内核＋C++设备封装”。前面的C函数继续使用，后续ROS 2的rclcpp节点复用同一个C++类。C++不会自动改善舵机精度，也不会让普通Linux变成硬实时控制器。

**操作分工：电脑WSL编辑、编译；RK连接ZLink并执行一次只读查询。** 本课无需source ROS、Python虚拟环境或colcon。默认运行程序只显示用法；只有显式给出端口及编号才打开设备，没有运动接口。

## 1. 文件与阅读顺序

| 文件 | 本课作用 |
| --- | --- |
| [read_position.cpp](../../native/apps/read_position.cpp) | main入口，校验命令行，创建串口对象，打印P值 |
| [position_reader.hpp](../../native/arm_driver/include/position_reader.hpp) | 类的接口：拥有串口，提供read_position |
| [position_reader.cpp](../../native/arm_driver/src/position_reader.cpp) | 打开/配置/关闭串口，完整写入与有期限接收 |
| [servo_codec.h](../../native/servo_protocol/include/servo_codec.h) | 前两课的C组帧与完整回复校验，保持原接口 |
| [servo_receiver.h](../../native/servo_protocol/include/servo_receiver.h) | 第三课的跨read半包/粘包接收器 |
| [build_position_reader.sh](../../tools/build_position_reader.sh) | 两台Linux都能用的编译脚本，只编译 |
| [test_cpp_position_reader.py](../../native/tests/test_cpp_position_reader.py) | 开发者使用的本地PTY集成检查，独立于实物 |

在**电脑WSL**的`~/Jixiebi`中打开文件，例如：

```bash
cd "$HOME/Jixiebi"
# 先读调用关系，再看类接口与实现；nano中Ctrl+O、Enter保存，Ctrl+X退出。
nano native/apps/read_position.cpp
```

参考实现已在这些文件中，练习时修改同名代码，不在文件尾再复制一份main或函数。先理解第2～6节的主链，poll/异常的细节留到第7节，不要求一次背下所有实现。

## 2. 主函数：把三个阶段串起来

**这两行已经在native/apps/read_position.cpp的main内部、try语句块中，不需要再复制到其他文件。** main从第32行开始，两行调用当前位于第72～74行；position_reader.hpp声明类接口，position_reader.cpp定义具体行为。头文件本身不会自动执行函数，程序运行时从main进入。

实际调用的核心是：

```cpp
// port来自--port，pty_simulation默认false；构造成功后才能查询。
jixiebi::PositionReader reader(port, pty_simulation);
const auto position_p = reader.read_position(
    static_cast<std::uint8_t>(servo_id), std::chrono::milliseconds(timeout_ms));
```

`reader`是对象，里面保存已打开的文件描述符。`reader.read_position(...)`调用对象的成员函数。`auto`让编译器从返回值推导类型，这里是`std::uint16_t`，并不是运行时不确定类型。`std::chrono::milliseconds`明确这个数字以毫秒计，默认500ms。

第一行按“类型、变量名、创建参数”拆开：jixiebi是命名空间，双冒号::在其中查找PositionReader类，reader是我们创建的对象名。括号中的port和pty_simulation会传给构造函数，不是查询参数；构造只准备串口，不发送PRAD查询。

第二行中的点号.表示使用reader对象的成员函数read_position；传入的是舵机编号和等待上限。static_cast<std::uint8_t>在编号已经检查为0～5后转换为8位无符号类型；std::chrono::milliseconds(timeout_ms)把整数明确包装成毫秒时长。const表示本次保存的position_p不再被修改；不是不能再次查询其他舵机。

例如执行`./build/cpp_lesson04/read_position --port /dev/ttyUSB0 --id 2`时，main通过argc/argv取得终端传入参数：

| 变量 | 值及来源 | 交给谁 |
| --- | --- | --- |
| port | --port后面的/dev/ttyUSB0 | 第一行构造函数 |
| pty_simulation | 默认false，没有加--pty-simulation | 第一行构造函数 |
| servo_id | --id后面的2，先校验再转类型 | 第二行read_position |
| timeout_ms | 未指定时为500 | 第二行read_position |

500ms是本次读写的等待上限，不是固定睡眠半秒；匹配回复提前到达就提前返回。read_position中的return只把P交还main，main随后打印SOURCE/SERVO_ID/POSITION_P；离开作用域再析构reader并关闭设备。main返回才结束整个程序。

先用int解析完整文本并检查ID0～5，再转uint8_t。不能先将256转为8位整数后再检查，否则会截断成0。输出POSITION_P仍是协议P单位，不能直接作为角度、坐标或准确位置。

## 3. C++类与资源生命周期

```cpp
class PositionReader {
public:
    explicit PositionReader(const std::string &port, bool pty_simulation = false);
    ~PositionReader();
    PositionReader(const PositionReader &) = delete;
    PositionReader &operator=(const PositionReader &) = delete;
private:
    int fd_ = -1;
};
```

`public`部分供调用者使用，`private`部分由类内部管理。`const std::string &`表示只读引用，不复制整个路径字符串。构造函数与类同名，创建对象时执行；`~PositionReader()`是析构函数，正常离开作用域或异常展开时关闭设备。这种由对象生命周期管理资源的方法叫RAII。默认信号终止不保证执行C++析构，进程退出仍由内核关闭描述符；不把程序退出等同物理停止。

`fd_`是Linux文件描述符：成功打开时是非负整数，-1表示未持有。删除复制操作是防止两个对象误以为各自拥有同一个描述符并重复关闭。本课对象由一个调用者使用，不提供多线程同步。

## 4. 打开、独占、配置

```cpp
fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
// 实际代码先检查open返回值，再在try中拿锁，成功后才配置。
if (::flock(fd_, LOCK_EX | LOCK_NB) == -1) {
    throw_system_error("flock: 串口已占用");
}
configure_port(fd_, pty_simulation);
```

`::open`指全局命名空间中的Linux接口，C++也能直接调用C/POSIX函数。O_RDWR允许收发，O_NOCTTY不把设备变成控制终端，O_NONBLOCK避免read/write无限等待，O_CLOEXEC避免执行其他程序时泄漏描述符。

flock与现有Python的exclusive=True采用同一协作锁，忙时失败，不抢占。构造中任何步骤抛异常都会先close再抛出，因为构造失败时析构函数不会执行。该锁不能阻止不遵守锁的外部程序，原ROS状态节点/旧控制程序仍须先退出。

configure_port使用termios设置115200、8数据位、无校验、1停止位，关闭软硬流控及行缓冲/本地回显，VMIN=1、VTIME=0，等待由poll和总期限负责。真实设备设置DTR/RTS为低，失败会明确报错；清除HUPCL不等于控制线已低，open瞬间也可能改变电平，不能承诺设备绝不复位。[termios接口说明](https://man7.org/linux/man-pages/man3/termios.3.html)

## 5. 发送：复用第一课，只发9字节

```cpp
char query[SERVO_POSITION_QUERY_CAPACITY]{};
if (servo_make_position_query(servo_id, query, sizeof(query)) != SERVO_CODEC_OK) {
    throw std::invalid_argument("舵机编号必须为0～5");
}
// 实际函数先初始化接收器、清旧输入，再统一设置本次deadline。
write_query(fd_, query, SERVO_POSITION_QUERY_LENGTH, deadline);
```

编号2生成`#002PRAD!`。query容量10，发送长度9，末尾C字符串的\0不在线上传输。write可能只写入一部分，write_query用sent记进度，只补未写入的尾部，不重新发送整条查询。成功写入只说明交给了系统，不能直接宣布得到舵机位置。

清旧输入的tcflush只放在发出前，放在发出后可能误删快速回复。原协议没有序号/设备时钟，因此清旧输入也不能排除“同ID在途迟到回复”；本课不声称解决了这一协议局限。

## 6. 接收：真实read接到第三课缓冲区

下列摘录省略错误分支；同名源码中保留了完整检查：

```cpp
// receiver定义和init位于while之前，仅初始化一次。
const ssize_t count = ::read(fd_, chunk, sizeof(chunk));
// 实际代码先检查count>0，再把实际收到的每个字节交给C接收器。
for (ssize_t index = 0; index < count; ++index) {
    const auto event = servo_frame_receiver_push(
        &receiver, static_cast<std::uint8_t>(chunk[index]));
    if (event == SERVO_RX_FRAME_READY) {
        std::uint16_t position_p = 0U;
        const auto status = servo_parse_position_reply(
            receiver.buffer, receiver.used, servo_id, &position_p);
        // 实际代码还检查deadline；成功才交付position_p。
    }
}
```

ssize_t可以表示read错误时的-1；count是实际字节数，不能换成strlen。一次read可能半帧，也可能多帧；receiver跨read保留状态，每个READY就在当前循环内解析，不能等整个chunk喂完才取最后一帧。

与查询完全一致的9字节回显跳过。错ID、坏数字、坏长度等候选不能成功，但在原期限内继续等合法匹配回复；遇新#、溢出按第三课重同步。整个查询累计接收超过4096字节终止，这是本课主机处理预算，不是厂家限制。

## 7. poll、超时与异常

poll等待POLLIN可读或POLLOUT可写，等待期间不忙转CPU。EINTR表示被信号打断，EAGAIN表示当前暂时无法读写，都回到原期限内等待，不重新计时。错误或设备断开则异常退出。[poll接口说明](https://man7.org/linux/man-pages/man2/poll.2.html)

期限由steady_clock计算，默认500ms，范围50～5000ms。所有读写共用同一deadline；poll返回后及交付成功前也检查，避免调度延迟把超期回复当作成功。普通Linux的调度/系统调用仍有抖动，这不是硬实时承诺。程序没有自动重试查询，更没有运动重发。

## 8. 电脑WSL编译：不接设备

```bash
cd "$HOME/Jixiebi"
# 只编译；成功末尾为BUILD_OK: build/cpp_lesson04/read_position。
bash tools/build_position_reader.sh
# 只查看用法，不打开默认串口。
./build/cpp_lesson04/read_position --help
```

编译脚本中的两次gcc -std=c11 -c生成C目标文件；g++ -std=c++17再编译C++源码并链接。-I指定头文件搜索目录，-o指定输出；C头文件已有extern "C"，防止C++改变C函数的链接名称。**不要把原.c直接当C++编译。**

可选的开发者本地检查已经由本次交付运行，你不必为了进入RK查询再重复：

```bash
# 只连接测试自身创建的/dev/pts设备，不需要机械臂。
python3 native/tests/test_cpp_position_reader.py --binary build/cpp_lesson04/read_position
```

电脑编出的x86_64可执行文件不能直接拿到aarch64的RK运行，复制源码后在RK编译。

## 9. 从仓库源码部署到RK

先按[统一RK部署指南](RK_DEPLOYMENT.md)选择“RK直接克隆”或“电脑从源码生成学习包”方式。公开仓库不依赖作者电脑上预先准备的压缩包。两种方式都将源码放在RK的`$HOME/Jixiebi`，不会复制电脑编出的x86_64程序。

首次部署完成即可进入下一节；已有完整源码时直接编译，不反复换目录或覆盖自己的修改。

## 10. RK编译并查询一次

以下在**RK的SSH终端**执行。前置条件是已按部署指南取得完整源码，ZLink与独立舵机电源已经按厂商要求连接，且Linux权限有效。然后在RK编译：

```bash
# cd只切换当前目录；bash执行脚本，脚本使用RK自己的gcc/g++生成aarch64程序。
cd "$HOME/Jixiebi" &&
bash tools/build_position_reader.sh
```

应显示BUILD_OK: build/cpp_lesson04/read_position。若提示gcc/g++不存在，把输出发来，先补RK工具链，不重装ROS。编译成功后，确认ZLink接RK的USB Host、原舵机独立电源已开启，停止原串口所有者。先看设备节点：

```bash
# 只列出串口设备及权限，不打开设备；此前ZLink对应/dev/ttyUSB0。
ls -l /dev/ttyUSB*
```

若设备不存在，停下核对枚举，不能凭空假定还是ttyUSB0。确认有/dev/ttyUSB0，再在RK执行一次查询：

```bash
# 在RK执行；只查002，不发送位置目标，不使用--pty-simulation。
cd "$HOME/Jixiebi" &&
if fuser -s /dev/ttyUSB0; then
  fuser -v /dev/ttyUSB0
  echo '先在对应窗口Ctrl+C退出旧串口程序，再运行本查询'
else
  ./build/cpp_lesson04/read_position --port /dev/ttyUSB0 --id 2
fi
```

成功格式示例（1495只是演示，不保证实物当前值）：

```text
TX: #002PRAD!
RX: #002P1495!
SOURCE=SERIAL_TTY
SERVO_ID=2
POSITION_P=1495
```

程序一次返回后自动关闭设备。日常只需进入同目录，串口空闲时运行最后一条查询命令，不用重传/解包。没有权限时检查id是否含dialout并重新登录；不要chmod 777或常态sudo运行驱动。超时保留错误输出，核对供电/接线；TIOCMGET/SET失败说明控制线接口尚未支持，不能拿--pty-simulation绕过真实设备错误。

查询命令中的./表示从当前目录找程序，--port指定RK本地设备，--id 2指定编号002；不带--pty-simulation使用真实设备配置。fuser只检查占用，程序随后还会拿flock，不能把这次快照当成永久独占。查询返回后不自动循环或重发。无需source ROS或激活/退出Python虚拟环境，C++可执行程序不依赖它们。

## 11. 本课验收与下一步

开发者已完成电脑WSL GCC/G++11.4.0严格警告构建、15项PTY检查（包含termios配置、分片/粘包、错误候选、4096预算、无重发超时、断开、锁与CLI）。PTY不能验证真实波特率电气行为、DTR/RTS、CH340、RK/aarch64或舵机反馈。完整原始TX/RX日志、六轴调度和ROS节点仍属后续阶段，本课只有终端诊断。

作者曾在RK查询002成功，得到RX #002P1492!、POSITION_P=1492，并确认理解main调用。继续[C++第五课：结构化结果与六轴顺序查询](CPP_LESSON05_ARM_STATE.md)，再接rclcpp状态节点；单轴成功不等于六轴/运动或坐标验收。[ROS 2 C++接口](https://github.com/ros2/rclcpp)
