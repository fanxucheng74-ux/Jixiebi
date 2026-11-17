# C第二课：完整位置回包的校验与解析

下一课见[C第三课：半包与粘包接收缓冲区](C_LESSON03_RECEIVER.md)，本课解析函数由接收器交付完整帧后调用。

本节所有编辑、编译和运行都在电脑WSL的`~/Jixiebi`进行，不在RK执行，不启动ROS、不连接串口。只处理已经收完整的一条位置回复；字节流如何拼成帧留下一节。

## 1. 输入输出与两个不同长度

教学样例为`#002P1495!`，不是本轮真实采样。预期编号为2，解析成功后输出位置1495P。

```text
下标    0  1  2  3  4  5  6  7  8  9
字符    #  0  0  2  P  1  4  9  5  !
字段    头  舵机编号  标记  位置数字   尾
```

| 内容 | 协议字节数 | 写成C字符串时的数组大小 |
| --- | --- | --- |
| 查询`#002PRAD!` | 9 | 10，包含末尾的\0 |
| 回复`#002P1495!` | 10 | 11，包含末尾的\0 |

解析函数的`length`是收到的有效字节数，不是缓冲区容量。字符串数组可用`sizeof(reply)-1U`排除结束标记；将来串口读取时用实际收到的长度，不能用接收数组的`sizeof`替代。函数本身不调用strlen，也不要求输入以\0结尾；调用者须保证至少有length个可读字节。

## 2. 在哪里写

本课参考实现已放入以下文件；对照手写时替换对应函数，不在同一文件重复定义一次。

- [servo_codec.h](../../native/servo_protocol/include/servo_codec.h)：增加回复长度、P数字范围、三个新错误码与函数声明。
- [servo_codec.c](../../native/servo_protocol/src/servo_codec.c)：在已有查询函数后新增完整回包解析，不删除第一课函数。
- [reply_parse_demo.c](../../native/apps/reply_parse_demo.c)：新演示入口，构造已知字符串、调用并打印。
- [test_servo_reply.c](../../native/tests/test_servo_reply.c)：开发者边界测试，读完本课再看即可。

若当前在Windows PowerShell，先执行`wsl`。之后在电脑WSL进入项目：

```bash
cd "$HOME/Jixiebi"
# 每次只打开一个文件；Ctrl+O、Enter保存，Ctrl+X退出后再打开下一个。
nano native/servo_protocol/include/servo_codec.h
```

退出头文件后打开实现：

```bash
nano native/servo_protocol/src/servo_codec.c
```

## 3. 声明与错误码

```c
servo_codec_status_t servo_parse_position_reply(
    const char *frame, size_t length, uint8_t expected_id, uint16_t *position_p);
```

- `frame`只读输入；`const`表示这个接口不修改回复字符。
- `length`有效帧长度，单位字节；当前必须为10。
- `expected_id`本次等待的舵机编号，仍只允许0～5。它是查询对象，不是最大编号；外部文本输入先验证数值再转换类型。
- `position_p`指向调用者的输出变量。调用时用`&position_p`取地址，成功后通过`*position_p`写回数值。
- 返回值是状态。失败时输出保持原值，调用者必须检查成功后才使用它，不能把保留的旧值当本次新读数。

新状态在旧枚举末尾追加，第一课的状态数值不变：

| 状态 | 当前数值 | 含义 |
| --- | --- | --- |
| SERVO_CODEC_OK | 0 | 全部通过，输出已更新 |
| SERVO_CODEC_INVALID_ARGUMENT | 1 | 输入/输出指针为空 |
| SERVO_CODEC_ID_OUT_OF_RANGE | 2 | 编号超出0～5 |
| SERVO_CODEC_FORMAT_ERROR | 4 | 固定字符错误或数字字段出现非数字 |
| SERVO_CODEC_LENGTH_ERROR | 5 | 帧不是恰好10个字节 |
| SERVO_CODEC_ID_MISMATCH | 6 | 收到的编号不是本次等待编号 |
| SERVO_CODEC_POSITION_OUT_OF_RANGE | 7 | P值超出协议数字范围500～2500 |

使用代码中的枚举名称，不在程序里直接写这些数字。P范围不是机械限位，也不是角度。

## 4. 逐段写检查与转换

第一段检查`frame == NULL || position_p == NULL`，拒绝空指针；再验证期待编号。`||`表示逻辑或，任意一边成立就拒绝。

第二段先检查长度：

```c
if (length != SERVO_POSITION_REPLY_LENGTH) {
    return SERVO_CODEC_LENGTH_ERROR;
}
```

这必须放在访问`frame[4]`、`frame[9]`之前。短包没有这些有效下标，不能先读它们再判断长度。长包、两帧拼在一起、帧后带回车换行也不属于这一条固定帧。

第三段检查固定字符：`frame[0]`为`#`，`frame[4]`为`P`，`frame[9]`为`!`。实现中的宏`SERVO_REPLY_ID_END`为4，`SERVO_POSITION_REPLY_LENGTH-1U`为9；源文件顶部的字段常量使范围有名称。

第四段解析编号：

```c
// BEGIN=1，END=4；从1开始，index<4，实际访问下标1、2、3。
uint16_t parsed_id = 0U;
for (size_t index = SERVO_REPLY_ID_BEGIN; index < SERVO_REPLY_ID_END; ++index) {
    if (frame[index] < '0' || frame[index] > '9') {
        return SERVO_CODEC_FORMAT_ERROR;
    }
    const uint16_t digit = (uint16_t)(frame[index] - '0');
    parsed_id = (uint16_t)(parsed_id * SERVO_DECIMAL_BASE + digit);
}
```

`SERVO_DECIMAL_BASE`是10。小于字符`'0'`或大于字符`'9'`就不是十进制数字，例如`A`、空格、冒号和\0都拒绝。检查之后，`'2'-'0'`才转换为数值2；每读一位，旧值乘10再加新数字。

三位编号先存`uint16_t`，可容纳0～999，检查不超过5后再匹配expected_id。如果先缩成8位，256可能变成0。收到`#004P1495!`但等待2时，应返回ID_MISMATCH，不误把004的反馈当002。

第五段对位置字段做同样处理：起点5、终点9，实际访问下标5～8。四个字符`1、4、9、5`依次得到`1→14→149→1495`，转换前逐位检查数字。四位十进制最多9999，不会溢出16位无符号整数；之后再限制协议范围500～2500。

最后才提交：

```c
// 前面全通过，才向调用者的变量写入结果。
*position_p = parsed_position;
return SERVO_CODEC_OK;
```

不要把输出赋值提前到中间某一步，否则错误帧可能留下半成品结果。本函数只验证格式与数值，没有厂家未提供的CRC或请求序号，也不能区分全部同ID迟到回复。

## 5. main调用与编译运行

电脑WSL打开演示：

```bash
nano native/apps/reply_parse_demo.c
```

主要调用是：

```c
const char reply[] = "#002P1495!";
const uint8_t expected_id = 2U;
uint16_t position_p = 0U;
const servo_codec_status_t status =
    servo_parse_position_reply(reply, sizeof(reply) - 1U, expected_id, &position_p);
```

数组reply有11字节，所以减1后传10；取地址符`&`让函数能够写回position_p。完整main已在文件中，包含状态检查；失败时打印错误码并退出，成功才打印解析值。

保存退出后，在电脑WSL的项目目录编译：

```bash
mkdir -p build/native_lesson02
# 复用同一个协议实现，仅将main入口换成本节的回包演示。
gcc -std=c11 -Wall -Wextra -Wpedantic -Werror \
  -I native/servo_protocol/include \
  native/servo_protocol/src/servo_codec.c \
  native/apps/reply_parse_demo.c \
  -o build/native_lesson02/reply_parse_demo
```

编译无报错后运行；若编译失败先停下，不运行上次遗留的旧程序：

```bash
./build/native_lesson02/reply_parse_demo
```

预期：

```text
回复帧: #002P1495!
匹配编号: 002
位置反馈: 1495 P
```

## 6. 亲手验证三个拒绝条件

每次只改一处，保存、重新编译、运行，观察后还原，再改下一处。

1. 将回复改成`#002P14A5!`：长度仍10，但A不是数字，错误码4。
2. 保持回复`#002P1495!`，将expected_id改成4U：帧本身正确，但编号不匹配，错误码6。
3. 保持原回复和编号，将调用中的`sizeof(reply)-1U`改成`sizeof(reply)`：把末尾\0计入，长度变11，错误码5。

不需要把每种坏帧都写成新的生产程序；这些是理解检查为何存在的小练习。

## 7. 已完成验证与范围

电脑WSL以GCC11.4.0严格警告构建成功，演示输出符合预期；单帧解析测试与第一课查询回归均通过，启用ASan/UBSan。覆盖没有NUL的10字节数组、短包、所有数字位错误、NULL、长度/编号/P边界、ID256缩窄风险和失败输出不变。

公开仓库可用`bash tools/check_native.sh`重现C解析检查与PTY测试，不需要作者的本地实验记录。本轮无RK、串口或相机访问，不将样例位置1495P写成实物状态。下一节再接有界流接收，处理一次read只有半帧或同时收到多帧的问题。
