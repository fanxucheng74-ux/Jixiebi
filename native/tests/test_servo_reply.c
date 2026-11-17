#include "servo_codec.h"

#include <assert.h>
#include <string.h>

static void expect_failure(const char *frame, size_t length, uint8_t expected_id,
                           servo_codec_status_t expected_status)
{
    uint16_t output = UINT16_MAX;
    const servo_codec_status_t status =
        servo_parse_position_reply(frame, length, expected_id, &output);
    assert(status == expected_status);
    assert(output == UINT16_MAX);  // 所有失败路径必须保留原输出。
}

static void test_valid_ids_and_positions(void)
{
    static const char *const frames[] = {
        "#000P1495!", "#001P1495!", "#002P1495!",
        "#003P1495!", "#004P1495!", "#005P1495!"
    };
    for (uint8_t id = 0U; id <= SERVO_ID_MAX; ++id) {
        uint16_t position = 0U;
        const servo_codec_status_t status =
            servo_parse_position_reply(frames[id], SERVO_POSITION_REPLY_LENGTH, id, &position);
        assert(status == SERVO_CODEC_OK);
        assert(position == 1495U);
    }

    uint16_t position = 0U;
    servo_codec_status_t status = servo_parse_position_reply("#002P0500!", 10U, 2U, &position);
    assert(status == SERVO_CODEC_OK && position == SERVO_POSITION_MIN_P);
    status = servo_parse_position_reply("#002P2500!", 10U, 2U, &position);
    assert(status == SERVO_CODEC_OK && position == SERVO_POSITION_MAX_P);

    // 恰10个字节、没有\0的原始数组也能解析，防止实现偷用strlen/sscanf越界。
    const char raw[] = {'#', '0', '0', '2', 'P', '1', '4', '9', '5', '!'};
    status = servo_parse_position_reply(raw, sizeof(raw), 2U, &position);
    assert(status == SERVO_CODEC_OK && position == 1495U);
}

static void test_lengths_and_null_pointers(void)
{
    const char reply[] = "#002P1495!";
    for (size_t length = 0U; length < SERVO_POSITION_REPLY_LENGTH; ++length) {
        expect_failure(reply, length, 2U, SERVO_CODEC_LENGTH_ERROR);
    }
    expect_failure(reply, sizeof(reply), 2U, SERVO_CODEC_LENGTH_ERROR);
    expect_failure(reply, SIZE_MAX, 2U, SERVO_CODEC_LENGTH_ERROR);
    expect_failure("#002PRAD!", 9U, 2U, SERVO_CODEC_LENGTH_ERROR);
    expect_failure("#002P1495!\r\n", 12U, 2U, SERVO_CODEC_LENGTH_ERROR);
    const char short_frame[] = {'#'};
    expect_failure(short_frame, sizeof(short_frame), 2U, SERVO_CODEC_LENGTH_ERROR);
    expect_failure(NULL, 10U, 2U, SERVO_CODEC_INVALID_ARGUMENT);
    expect_failure(NULL, 0U, 2U, SERVO_CODEC_INVALID_ARGUMENT);

    const servo_codec_status_t status = servo_parse_position_reply(reply, 10U, 2U, NULL);
    assert(status == SERVO_CODEC_INVALID_ARGUMENT);
}

static void test_format_and_decimal_fields(void)
{
    expect_failure("?002P1495!", 10U, 2U, SERVO_CODEC_FORMAT_ERROR);
    expect_failure("#002X1495!", 10U, 2U, SERVO_CODEC_FORMAT_ERROR);
    expect_failure("#002P1495?", 10U, 2U, SERVO_CODEC_FORMAT_ERROR);
    expect_failure(" #002P1495", 10U, 2U, SERVO_CODEC_FORMAT_ERROR);

    static const size_t digit_indices[] = {1U, 2U, 3U, 5U, 6U, 7U, 8U};
    static const char bad_digits[] = {'/', ':', 'A', ' ', '\0', (char)0xFF};
    for (size_t field = 0U; field < sizeof(digit_indices) / sizeof(digit_indices[0]); ++field) {
        for (size_t value = 0U; value < sizeof(bad_digits) / sizeof(bad_digits[0]); ++value) {
            char frame[] = "#002P1495!";
            frame[digit_indices[field]] = bad_digits[value];
            expect_failure(frame, sizeof(frame) - 1U, 2U, SERVO_CODEC_FORMAT_ERROR);
        }
    }
}

static void test_id_matching_and_position_range(void)
{
    expect_failure("#004P1495!", 10U, 2U, SERVO_CODEC_ID_MISMATCH);
    expect_failure("#002P1495!", 10U, 6U, SERVO_CODEC_ID_OUT_OF_RANGE);
    expect_failure("#002P1495!", 10U, UINT8_MAX, SERVO_CODEC_ID_OUT_OF_RANGE);
    expect_failure("#006P1495!", 10U, 2U, SERVO_CODEC_ID_OUT_OF_RANGE);
    expect_failure("#255P1495!", 10U, 0U, SERVO_CODEC_ID_OUT_OF_RANGE);
    expect_failure("#256P1495!", 10U, 0U, SERVO_CODEC_ID_OUT_OF_RANGE);
    expect_failure("#999P1495!", 10U, 2U, SERVO_CODEC_ID_OUT_OF_RANGE);
    expect_failure("#002P0499!", 10U, 2U, SERVO_CODEC_POSITION_OUT_OF_RANGE);
    expect_failure("#002P2501!", 10U, 2U, SERVO_CODEC_POSITION_OUT_OF_RANGE);
    expect_failure("#002P0000!", 10U, 2U, SERVO_CODEC_POSITION_OUT_OF_RANGE);
    expect_failure("#002P9999!", 10U, 2U, SERVO_CODEC_POSITION_OUT_OF_RANGE);
}

int main(void)
{
    test_valid_ids_and_positions();
    test_lengths_and_null_pointers();
    test_format_and_decimal_fields();
    test_id_matching_and_position_range();
    return 0;
}
