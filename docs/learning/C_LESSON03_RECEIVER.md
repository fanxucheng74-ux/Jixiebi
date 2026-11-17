# C第三课：半包与粘包的有界接收缓冲区

本节在电脑WSL的`~/Jixiebi`编辑、编译和运行，不在RK执行，不调用串口/相机或ROS。上一课已实现完整帧校验，本节复用已有完整帧解析器，不修改它。

## 1. 为什么要在解析函数前增加接收器

一次read得到的是一段字节，不保证恰好对应一条回复。模拟半包：第一次得到`#002P14`，第二次得到`95!`；模拟粘包：一次得到`#002P1500!#002P1510!`。

```text
每次到达的字节片段 → 保留跨次状态、按#和!分帧 → 完整候选帧 → 上一课的内容校验
```

接收器只负责“从哪里开始、在哪里结束”。即使得到`#!`或`#002P14A5!`，也只是一个界定完整的候选，长度/数字等仍由上一课函数拒绝。分帧成功不表示已得到合法、新鲜的位置数据。

## 2. 文件与操作位置

- [servo_receiver.h](../../native/servo_protocol/include/servo_receiver.h)：缓冲区结构体、分帧事件、函数声明。
- [servo_receiver.c](../../native/servo_protocol/src/servo_receiver.c)：初始化和逐字节接收。
- [receiver_demo.c](../../native/apps/receiver_demo.c)：模拟三次输入，立即解析每个完整帧。
- [test_servo_receiver.c](../../native/tests/test_servo_receiver.c)：开发者边界检查，不要求先读。

参考实现已存在，对照编写时替换同名函数，不重复定义。若当前在Windows PowerShell，先执行`wsl`。以下命令都在电脑WSL执行：

```bash
cd "$HOME/Jixiebi"
# 先看类型和接口，Ctrl+O、Enter保存，Ctrl+X退出后再打开下一个文件。
nano native/servo_protocol/include/servo_receiver.h
```

```bash
nano native/servo_protocol/src/servo_receiver.c
```

## 3. 接收状态为什么放在结构体

```c
typedef struct {
    char buffer[SERVO_RECEIVER_BUFFER_CAPACITY];
    size_t used;
    bool frame_ready;
} servo_frame_receiver_t;
```

- `buffer`保存已收集字节。当前数组65字节，最多64个协议字节加结尾\0；64是我们选的有界内存策略，不是厂家上限。实际位置回复仍为10字节。
- `used`是已经保存的协议字节数，也是下一字节的写入下标，不包含补的\0。
- `frame_ready`是bool，只有true/false：刚收到帧尾!时为true，表示当前候选帧可处理。

接收状态由main持有，多次调用都传同一个地址`&receiver`。函数中的`receiver->used`表示访问这个指针所指对象的used成员，等价于`(*receiver).used`；main持有对象本身时可写`receiver.used`。

不能把接收状态定义在每次输入就重新创建的局部对象中，否则半帧会随调用结束而丢失。也不能每来一个chunk就init。

## 4. 初始化只做一次

```c
bool servo_frame_receiver_init(servo_frame_receiver_t *receiver)
{
    if (receiver == NULL) {
        return false;
    }
    receiver->used = 0U;
    receiver->buffer[0] = '\0';
    receiver->frame_ready = false;
    return true;
}
```

开始时还没有收到帧头，有效长度为0，显示为空字符串，也没有完整帧。只清空有效长度和首字符即可；数组剩余旧字节不属于有效数据。

调用前必须初始化。push中对used的防御检查不能检测所有未初始化对象，不能用它代替init。

## 5. push每次只处理一个字节

完整实现位于servo_receiver.c，按以下顺序阅读。

1. 拒绝空指针和used超过内存上限的状态，防止越界。
2. 如果上一帧已经交付，将当前有效长度和ready标记清空，从本字节开始复用缓冲区。上一帧须在这一步之前处理或复制。
3. 如果是新`#`，启动新帧：buffer[0]写#、used置1、后面补\0。旧半帧存在时报告RESTARTED。这个分支放在容量判断前，让占满的坏半帧也能被新帧头打断。
4. 如果used为0且当前不是#，忽略帧外噪声，包括额外的!。
5. 如果used已达64，不能再追加：丢弃当前超长片段、等待下一个#，报告OVERFLOW。
6. 否则追加一个字节，used加1，再补\0。
7. 如果追加的是!，标记ready并交付；其他字节继续等待。

核心写入是：

```c
// used既记录已收字节数，也指出下一写入位置。
receiver->buffer[receiver->used] = (char)byte;
receiver->used += 1U;
receiver->buffer[receiver->used] = '\0';
```

容量检查必须在前面。used为63时，写下标63后变64，结束标记写下标64，均在65字节数组内。不能把\0也算成协议字节。

| 事件 | 含义 |
| --- | --- |
| SERVO_RX_NONE | 暂无完整帧，可能在等待，也可能刚忽略了噪声 |
| SERVO_RX_FRAME_READY | 发现#…!候选帧，立刻交给解析器 |
| SERVO_RX_RESTARTED | 新#替换未结束的旧半帧 |
| SERVO_RX_OVERFLOW | 旧片段过长，已丢弃，等待新# |
| SERVO_RX_INVALID_ARGUMENT | 无效参数或部分非法状态，调用者需要处理 |

本实现是固定数组分帧状态机，没有内部多帧队列、线程同步或超时逻辑。后续事务层会管理超时。收到原始0字节时used仍正常计数；不能用strlen推测实际输入长度。

## 6. 半包怎样保留

```text
第一次：#002P14 → used=7，ready=false → 保留，不调用完整帧解析器
第二次：95!     → 继续写入 → #002P1495! → used=10，ready=true → 立即解析
```

只有第一字节#启动收集；两次调用之间不清空同一个receiver。补齐后才交给上一课，因此不会误把第一段7字节送去完整帧解析后报长度错误。

## 7. 粘包怎样逐个交付

演示循环的关键片段：

```c
for (size_t index = 0U; index < length; ++index) {
    const servo_rx_event_t event =
        servo_frame_receiver_push(receiver, (uint8_t)chunk[index]);

    if (event == SERVO_RX_FRAME_READY) {
        uint16_t position_p = 0U;
        const servo_codec_status_t status = servo_parse_position_reply(
            receiver->buffer, receiver->used, 2U, &position_p);
        // 检查status后再使用position_p；完整程序同时检查其他事件和打印结果。
    }
}
```

遇到第一条!就立即处理第一帧；下一个字节#开始第二帧，再遇!就处理第二帧。必须在for循环里面响应READY，不能把整个chunk喂完后才看buffer，否则第一帧已经被第二帧覆盖。

FRAME_READY的数据只在下一次push/init前有效；要异步处理，必须先复制到自己持有的存储，不能只存下指向同一个buffer的多个指针。

## 8. 演示输入与编译运行

电脑WSL打开演示：

```bash
nano native/apps/receiver_demo.c
```

main只初始化一次，然后按顺序喂入：

```c
const char first[] = "#002P14";
const char second[] = "95!";
const char third[] = "#002P1500!#002P1510!";
```

本演示的chunk是以\0结束的固定字符串，方便打印；传入长度均用sizeof(array)-1，不将\0喂入接收器。以后真实read返回n字节时，就送入n字节，无需等到字符串结束标记。

在电脑WSL项目目录编译：

```bash
mkdir -p build/native_lesson03
# 这次加入接收器实现；同一可执行文件只编译本节的一个main。
gcc -std=c11 -Wall -Wextra -Wpedantic -Werror \
  -I native/servo_protocol/include \
  native/servo_protocol/src/servo_codec.c \
  native/servo_protocol/src/servo_receiver.c \
  native/apps/receiver_demo.c \
  -o build/native_lesson03/receiver_demo
```

编译成功再运行，编译报错时先停下，不运行遗留的旧程序：

```bash
./build/native_lesson03/receiver_demo
```

预期输出：

```text
收到片段: #002P14
保留半帧: #002P14（7字节）
收到片段: 95!
完整帧: #002P1495! -> 位置=1495 P
收到片段: #002P1500!#002P1510!
完整帧: #002P1500! -> 位置=1500 P
完整帧: #002P1510! -> 位置=1510 P
```

## 9. 你亲手改动的练习

保持third不变，把前两段改为：

```c
const char first[] = "#002P1";
const char second[] = "495!";
```

重新编译运行。第一次应保留6字节，最后仍得到1495P，因为片段边界不影响完整数据。先完成这个练习，再可尝试将third第二帧的1510改成1520，应按顺序输出1500P、1520P。

## 10. 已完成的开发者验证

电脑WSL GCC11.4.0严格警告编译，ASan/UBSan接收器边界检查和旧两课C回归通过。覆盖所有两段切分位置、逐字节、三帧粘连、尾半帧、噪声/新#恢复、64/65字节边界、溢出后恢复、嵌入NUL/高位字节、双实例独立及业务解析拒绝。

公开仓库可用`bash tools/check_native.sh`执行接收器边界检查与PTY测试。没有RK、串口/相机或实物状态/运动验收。作者已确认本课半包和粘包示例跑通；继续[C++第四课：串口只读](CPP_LESSON04_SERIAL.md)，用真实read返回长度喂同一个接收器，先WSL编译，再由用户在RK查询。
