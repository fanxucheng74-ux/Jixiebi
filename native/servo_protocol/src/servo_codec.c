#include "servo_codec.h"

#include <stdio.h>  // snprintf按容量限制格式化输出，返回值不包含结尾\0。

// 半开区间[起点,终点)对应固定协议字段：编号下标1～3，P字段下标5～8。
#define SERVO_REPLY_ID_BEGIN (1U)
#define SERVO_REPLY_ID_END (4U)
#define SERVO_REPLY_POSITION_BEGIN (5U)
#define SERVO_REPLY_POSITION_END (9U)
#define SERVO_DECIMAL_BASE (10U)

servo_codec_status_t servo_make_position_query(uint8_t servo_id, char *buffer, size_t capacity)
{
    // 先检查指针与容量，再访问buffer[0]，避免空指针或零容量写入。
    if (buffer == NULL) {
        return SERVO_CODEC_INVALID_ARGUMENT;
    }
    if (capacity == 0U) {
        return SERVO_CODEC_BUFFER_TOO_SMALL;
    }

    // 参数不合法时也不会遗留一个看起来可发送的旧查询字符串。
    buffer[0] = '\0';
    if (servo_id > SERVO_ID_MAX) {
        return SERVO_CODEC_ID_OUT_OF_RANGE;
    }
    if (capacity < SERVO_POSITION_QUERY_CAPACITY) {
        return SERVO_CODEC_BUFFER_TOO_SMALL;
    }

    // %03u表示至少三位十进制、不足补0；前面的范围检查才负责拒绝非法编号。
    // 显式转为unsigned int以匹配%u的标准库参数类型，避免可变参数类型不匹配。
    const int written = snprintf(buffer, capacity, "#%03uPRAD!", (unsigned int)servo_id);
    if (written != (int)SERVO_POSITION_QUERY_LENGTH) {
        buffer[0] = '\0';
        return SERVO_CODEC_FORMAT_ERROR;
    }

    return SERVO_CODEC_OK;
}

servo_codec_status_t servo_parse_position_reply(const char *frame, size_t length,
                                                uint8_t expected_id, uint16_t *position_p)
{
    // 输入只读，输出由调用者提供地址；两者都有效后才允许访问对应内存。
    if (frame == NULL || position_p == NULL) {
        return SERVO_CODEC_INVALID_ARGUMENT;
    }
    if (expected_id > SERVO_ID_MAX) {
        return SERVO_CODEC_ID_OUT_OF_RANGE;
    }

    // 必须先查实际长度再使用固定下标，防止短包访问frame[9]时越界。
    // 本函数只接受一帧，不接受帧后的\0、回车换行或另一条回复。
    if (length != SERVO_POSITION_REPLY_LENGTH) {
        return SERVO_CODEC_LENGTH_ERROR;
    }
    if (frame[0] != '#' || frame[SERVO_REPLY_ID_END] != 'P' ||
        frame[SERVO_POSITION_REPLY_LENGTH - 1U] != '!') {
        return SERVO_CODEC_FORMAT_ERROR;
    }

    // 先用16位保存三位十进制数；若先缩成uint8_t，256可能绕回0而被误认。
    uint16_t parsed_id = 0U;
    for (size_t index = SERVO_REPLY_ID_BEGIN; index < SERVO_REPLY_ID_END; ++index) {
        if (frame[index] < '0' || frame[index] > '9') {
            return SERVO_CODEC_FORMAT_ERROR;
        }
        const uint16_t digit = (uint16_t)(frame[index] - '0');
        parsed_id = (uint16_t)(parsed_id * SERVO_DECIMAL_BASE + digit);
    }
    if (parsed_id > SERVO_ID_MAX) {
        return SERVO_CODEC_ID_OUT_OF_RANGE;
    }
    if (parsed_id != expected_id) {
        return SERVO_CODEC_ID_MISMATCH;
    }

    // 四位十进制最多9999，uint16_t足够容纳；仍需逐字符检查，不能只转前缀。
    uint16_t parsed_position = 0U;
    for (size_t index = SERVO_REPLY_POSITION_BEGIN; index < SERVO_REPLY_POSITION_END; ++index) {
        if (frame[index] < '0' || frame[index] > '9') {
            return SERVO_CODEC_FORMAT_ERROR;
        }
        const uint16_t digit = (uint16_t)(frame[index] - '0');
        parsed_position = (uint16_t)(parsed_position * SERVO_DECIMAL_BASE + digit);
    }
    if (parsed_position < SERVO_POSITION_MIN_P || parsed_position > SERVO_POSITION_MAX_P) {
        return SERVO_CODEC_POSITION_OUT_OF_RANGE;
    }

    // 只有全部有效时才提交输出；收到坏包不会将半成品值覆盖调用者的旧状态。
    *position_p = parsed_position;
    return SERVO_CODEC_OK;
}
