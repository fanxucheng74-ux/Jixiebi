#include "servo_codec.h"

#include <assert.h>
#include <string.h>

static void test_all_project_ids(void)
{
    static const char *const expected[] = {
        "#000PRAD!", "#001PRAD!", "#002PRAD!", "#003PRAD!", "#004PRAD!", "#005PRAD!"
    };

    for (uint8_t servo_id = 0U; servo_id <= SERVO_ID_MAX; ++servo_id) {
        char frame[SERVO_POSITION_QUERY_CAPACITY];
        const servo_codec_status_t status =
            servo_make_position_query(servo_id, frame, sizeof(frame));
        assert(status == SERVO_CODEC_OK);
        assert(strcmp(frame, expected[servo_id]) == 0);
        assert(strlen(frame) == SERVO_POSITION_QUERY_LENGTH);
        assert(frame[SERVO_POSITION_QUERY_LENGTH] == '\0');
    }
}

static void test_capacity_boundaries(void)
{
    char storage[SERVO_POSITION_QUERY_CAPACITY + 2U];

    // 以哨兵检查函数没有写入声明容量之外，或失败后留下半截查询帧。
    memset(storage, 'X', sizeof(storage));
    servo_codec_status_t status =
        servo_make_position_query(4U, storage, SERVO_POSITION_QUERY_CAPACITY);
    assert(status == SERVO_CODEC_OK);
    assert(storage[SERVO_POSITION_QUERY_CAPACITY] == 'X');
    assert(storage[SERVO_POSITION_QUERY_CAPACITY + 1U] == 'X');

    memset(storage, 'X', sizeof(storage));
    status = servo_make_position_query(4U, storage, sizeof(storage));
    assert(status == SERVO_CODEC_OK);
    assert(strcmp(storage, "#004PRAD!") == 0);
    assert(storage[SERVO_POSITION_QUERY_CAPACITY] == 'X');

    for (size_t capacity = 1U; capacity < SERVO_POSITION_QUERY_CAPACITY; ++capacity) {
        memset(storage, 'X', sizeof(storage));
        status = servo_make_position_query(4U, storage, capacity);
        assert(status == SERVO_CODEC_BUFFER_TOO_SMALL);
        assert(storage[0] == '\0');
        for (size_t index = 1U; index < sizeof(storage); ++index) {
            assert(storage[index] == 'X');
        }
    }

    memset(storage, 'X', sizeof(storage));
    status = servo_make_position_query(4U, storage, 0U);
    assert(status == SERVO_CODEC_BUFFER_TOO_SMALL);
    assert(storage[0] == 'X');
}

static void test_invalid_arguments(void)
{
    char frame[SERVO_POSITION_QUERY_CAPACITY] = "old";
    servo_codec_status_t status = servo_make_position_query(6U, frame, sizeof(frame));
    assert(status == SERVO_CODEC_ID_OUT_OF_RANGE);
    assert(frame[0] == '\0');

    status = servo_make_position_query(UINT8_MAX, frame, sizeof(frame));
    assert(status == SERVO_CODEC_ID_OUT_OF_RANGE);
    assert(frame[0] == '\0');

    status = servo_make_position_query(4U, NULL, SERVO_POSITION_QUERY_CAPACITY);
    assert(status == SERVO_CODEC_INVALID_ARGUMENT);
    status = servo_make_position_query(4U, NULL, 0U);
    assert(status == SERVO_CODEC_INVALID_ARGUMENT);
}

int main(void)
{
    test_all_project_ids();
    test_capacity_boundaries();
    test_invalid_arguments();
    return 0;
}
