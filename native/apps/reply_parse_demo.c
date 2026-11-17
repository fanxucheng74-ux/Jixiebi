#include "servo_codec.h"

#include <stdio.h>
#include <stdlib.h>

// 固定的教学样例，不代表实物当前位置；本程序没有串口或机械臂操作。
int main(void)
{
    const char reply[] = "#002P1495!";
    const uint8_t expected_id = 2U;  // 本次查询对象，不是允许编号的最大值。
    uint16_t position_p = 0U;

    // sizeof(reply)为11；减1排除字符串结尾\0，传入实际协议帧的10字节。
    // &position_p取得变量地址，供函数通过输出指针写回结果。
    const servo_codec_status_t status =
        servo_parse_position_reply(reply, sizeof(reply) - 1U, expected_id, &position_p);
    if (status != SERVO_CODEC_OK) {
        (void)fprintf(stderr, "解析失败，错误码=%d\n", (int)status);
        return EXIT_FAILURE;
    }

    const int printed = printf("回复帧: %s\n匹配编号: %03u\n位置反馈: %u P\n",
                               reply, (unsigned int)expected_id, (unsigned int)position_p);
    if (printed < 0) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
