# C第一课：生成舵机位置查询帧

下一课见[C第二课：完整位置回包解析](C_LESSON02_REPLY.md)，本节查询函数继续保留。

本节在电脑的WSL编写和编译，不在RK执行，不需要启动ROS或连接机械臂。项目已提供这一小节的参考实现；请打开源码逐段对照、亲自修改，再重新编译。本节不加入接收解析、运动组帧或串口发送。

## 1. 目标与操作位置

输入编号`4`，生成字符串`#004PRAD!`，打印到WSL终端。生成字符串不等于向舵机发出了查询。

在电脑Windows PowerShell打开已有WSL：

```powershell
# 这条只在Windows执行；如果已进入WSL则跳过。
wsl
```

随后全部命令均在电脑WSL执行：

```bash
# 以下假设源码克隆在WSL家目录；若路径不同，替换为你的仓库目录。
cd "$HOME/Jixiebi"
```

开发者使用的WSL为x86_64、GCC 11.4.0；读者应核对自己的编译器。以后部署RK需在aarch64目标上重新编译或使用匹配交叉工具链，不能把本节x86_64可执行文件直接当RK程序。

## 2. 查询指令里的字符

```text
#    004    PRAD    !
帧头 编号   读位置  帧尾
```

当前工程允许编号0～5。`PRAD`沿用已核对的厂家位置查询指令，本节不自行设计新的舵机协议。

`#004PRAD!`共有9个ASCII字符，对应9个协议字节。C字符串还需第10个字节存放`'\0'`，因此数组容量至少10。`'0'`是可显示的数字字符，`'\0'`是数值为0的字符串结束标记，两者不同。未来串口发送只发送前9字节，不发送这个结束标记。

## 3. 三个核心文件的职责

| 文件 | 本节职责 |
| --- | --- |
| [servo_codec.h](../../native/servo_protocol/include/servo_codec.h) | 常量、状态枚举、函数声明与输入输出约定 |
| [servo_codec.c](../../native/servo_protocol/src/servo_codec.c) | 参数检查、格式化及返回状态，不连接设备 |
| [query_frame_demo.c](../../native/apps/query_frame_demo.c) | 准备数组，调用函数，检查结果并打印 |

开发者另有[边界测试](../../native/tests/test_servo_query.c)，不用先阅读它才能学习本节。先关注接口与调用过程；头文件保护和`extern "C"`分别用于避免重复包含和以后供C++调用，后续再深入。

在WSL查看实现与调用：

```bash
# 查看函数实现；也可在编辑器中逐段输入、修改参考代码。
nano native/servo_protocol/src/servo_codec.c
# 再查看调用者；后面的练习只改这个文件中的servo_id。
nano native/apps/query_frame_demo.c
```

在nano中按Ctrl+O、Enter保存，Ctrl+X退出。不要将两个编辑命令一次粘进仍打开的编辑器，先退出一个文件再打开另一个。

## 4. 函数参数怎样理解

```c
servo_codec_status_t servo_make_position_query(
    uint8_t servo_id, char *buffer, size_t capacity);
```

- `servo_codec_status_t`是头文件中的枚举类型，返回成功或具体错误，不是查询结果字符串。
- `servo_id`是编号，`uint8_t`表示8位无符号整数；本函数仍限制为0～5。以后从文本读取编号时须先验证数值再转类型，不能先把256转换后当成0。
- `buffer`是调用者数组首元素的地址；函数通过这个地址向调用者的数组写字符，不是创建并返回一个局部数组地址。
- `capacity`是该数组实际可写容量，单位字节。调用者必须如实传入，函数无法凭指针知道真实数组大小。
- `size_t`是标准库用于对象大小的无符号类型，应保留它，不机械改成固定宽度整数。

## 5. 按执行顺序阅读实现

第一段拒绝`NULL`和容量0，再写`buffer[0]`。先访问数组后判断指针会造成非法内存访问。

第二段把首字符设成`'\0'`，清空输出；随后拒绝ID大于5或容量不足10。这样失败后调用者不会误把旧字符串当新结果。容量0或空指针时不写任何内存。

第三段是核心组帧：

```c
// buffer是输出位置，capacity是最多可写字节数。
// %03u要求编号至少三位，不足补0；例如4变为004。
const int written = snprintf(buffer, capacity, "#%03uPRAD!", (unsigned int)servo_id);
```

`%u`对应`unsigned int`，因此这里明确转换类型；`03`仅是最小显示宽度，不能代替前面的ID校验。`snprintf`最多使用给定容量，并返回原本需要输出的字符数，返回值不包含结束标记。它的标准返回类型是`int`，出错可为负，所以此处保留`int`。

最后要求返回长度恰好为9；否则清空输出并返回格式错误。成功时返回`SERVO_CODEC_OK`，字符串留在调用者提供的数组中。

## 6. main怎样调用

```c
// 数组由main持有，函数只在调用期间通过指针使用它。
char frame[SERVO_POSITION_QUERY_CAPACITY];
const servo_codec_status_t status =
    servo_make_position_query(servo_id, frame, sizeof(frame));
```

`sizeof(frame)`在这里得到整个数组的10字节；如果进入函数后对参数`buffer`使用`sizeof`，得到的是指针自身大小，不能据此推算数组容量。

完整演示会先检查`status`；失败返回`EXIT_FAILURE`，成功才打印。`%s`打印字符串，`%zu`打印`size_t`。`strlen(frame)`得到9，`sizeof(frame)`得到10。`int main(void)`是标准程序入口，`return`值交给终端判断执行是否成功。

## 7. 编译与运行

在电脑WSL的`~/Jixiebi`执行：

```bash
# 构建产物单独存放；build目录已被Git忽略。
mkdir -p build/native_lesson01

# 将组帧实现和演示程序一起编译链接；本节不需要CMake或ROS。
gcc -std=c11 -Wall -Wextra -Wpedantic -Werror \
  -I native/servo_protocol/include \
  native/servo_protocol/src/servo_codec.c \
  native/apps/query_frame_demo.c \
  -o build/native_lesson01/query_frame_demo

# 仅在编译成功后运行；程序只打印字符串，不访问硬件。
./build/native_lesson01/query_frame_demo
```

`-std=c11`选择C11标准；警告选项帮助发现格式/类型等问题，`-Werror`使警告也阻止本节构建；`-I`提供头文件搜索目录，两个`.c`分别提供函数与main，`-o`指定可执行文件。

预期输出：

```text
查询帧: #004PRAD!
线上长度: 9 字节
缓冲区容量: 10 字节
```

若编译报错，停在编译这一步，不运行旧二进制；把完整报错用于排查。`gcc: command not found`或项目目录不存在时先核对当前是否在正确WSL和目录，不自动安装系统软件。本轮参考程序已在上述WSL实际编译运行。

## 8. 你亲手完成的小练习

在`query_frame_demo.c`将`const uint8_t servo_id = 4U;`改为`2U`，保存、重新执行编译和运行。修改前先预测：字符串中哪些字符会变，长度和容量是否改变？预期是`#002PRAD!`，长度9、容量10不变。

不要修改协议帧尾或加入运动指令。下一节再讲接收字节如何组成完整回包，以及为何一次read可能读不完整；串口发送仍在之后单独接入。

## 9. 本轮验证边界

已在电脑WSL以严格编译警告构建演示与C测试，边界测试同时启用AddressSanitizer/UndefinedBehaviorSanitizer。覆盖六个合法ID、ID6/255、NULL、容量0～9、较大容量和越界哨兵，结果为`C_QUERY_BOUNDARIES_PASS`。

本轮没有连接RK、打开串口/相机或验证实物通信；新函数只有组帧能力。公开仓库可用`bash tools/check_native.sh`执行C边界检查与PTY测试；不把本节测试等同于后续驱动和机械臂验收。
