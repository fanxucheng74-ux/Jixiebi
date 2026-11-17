#include "servo_codec.h"
#include "servo_receiver.h"

#include <stdio.h>
#include <stdlib.h>

// 仅用于本演示的已知字符串；未来串口输入应使用read实际返回的长度。
static int feed_chunk(servo_frame_receiver_t *receiver, const char *chunk, size_t length)
{
    if (receiver == NULL || chunk == NULL) {
        return EXIT_FAILURE;
    }
    if (printf("收到片段: %s\n", chunk) < 0) {
        return EXIT_FAILURE;
    }

    for (size_t index = 0U; index < length; ++index) {
        const servo_rx_event_t event =
            servo_frame_receiver_push(receiver, (uint8_t)chunk[index]);

        // 必须在循环内立即处理READY；下次push会开始复用同一个缓冲区。
        if (event == SERVO_RX_FRAME_READY) {
            uint16_t position_p = 0U;
            const uint8_t expected_id = 2U;
            const servo_codec_status_t status = servo_parse_position_reply(
                receiver->buffer, receiver->used, expected_id, &position_p);
            if (status == SERVO_CODEC_OK) {
                if (printf("完整帧: %s -> 位置=%u P\n", receiver->buffer,
                           (unsigned int)position_p) < 0) {
                    return EXIT_FAILURE;
                }
            } else {
                // 候选帧界定完整，内容仍可能被上一课的业务解析器拒绝。
                if (printf("候选帧被解析器拒绝，错误码=%d\n", (int)status) < 0) {
                    return EXIT_FAILURE;
                }
            }
        } else if (event == SERVO_RX_RESTARTED) {
            if (printf("新#到达，丢弃旧半帧并重新同步\n") < 0) {
                return EXIT_FAILURE;
            }
        } else if (event == SERVO_RX_OVERFLOW) {
            if (printf("帧过长，已丢弃，等待新的#\n") < 0) {
                return EXIT_FAILURE;
            }
        } else if (event == SERVO_RX_INVALID_ARGUMENT) {
            return EXIT_FAILURE;
        }
    }

    if (receiver->used > 0U && !receiver->frame_ready) {
        if (printf("保留半帧: %s（%zu字节）\n", receiver->buffer, receiver->used) < 0) {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}

int main(void)
{
    // 接收器放在各次feed_chunk之外，只初始化一次，才能保留跨次到达的半帧。
    servo_frame_receiver_t receiver;
    if (!servo_frame_receiver_init(&receiver)) {
        return EXIT_FAILURE;
    }

    const char first[] = "#002P14";
    const char second[] = "95!";
    const char third[] = "#002P1500!#002P1510!";

    if (feed_chunk(&receiver, first, sizeof(first) - 1U) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }
    if (feed_chunk(&receiver, second, sizeof(second) - 1U) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }
    if (feed_chunk(&receiver, third, sizeof(third) - 1U) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
