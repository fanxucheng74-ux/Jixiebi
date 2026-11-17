#include "servo_codec.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 本节入口只组帧并打印；没有open/read/write等设备访问，不会查询或移动实物。
int main(void)
{
    const uint8_t servo_id = 4U;  // 本次查询对象，不是最大编号；改成2U即选择002号。
    char frame[SERVO_POSITION_QUERY_CAPACITY];

    // frame传入时是数组首元素地址；sizeof(frame)在这里才是整个数组的字节容量。
    const servo_codec_status_t status = servo_make_position_query(servo_id, frame, sizeof(frame));
    if (status != SERVO_CODEC_OK) {
        // 即使错误信息无法打印，也必须向终端返回失败，不能继续使用frame。
        (void)fprintf(stderr, "组帧失败，错误码=%d\n", (int)status);
        return EXIT_FAILURE;
    }

    // strlen不统计结尾\0；sizeof数组则包含给\0预留的空间。
    const int printed = printf("查询帧: %s\n线上长度: %zu 字节\n缓冲区容量: %zu 字节\n",
                               frame, strlen(frame), sizeof(frame));
    if (printed < 0) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
