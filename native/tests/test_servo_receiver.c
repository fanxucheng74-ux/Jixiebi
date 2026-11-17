#include "servo_codec.h"
#include "servo_receiver.h"

#include <assert.h>
#include <string.h>

#define TEST_CAPTURE_CAPACITY (8U)

typedef struct {
    char frames[TEST_CAPTURE_CAPACITY][SERVO_RECEIVER_BUFFER_CAPACITY];
    size_t lengths[TEST_CAPTURE_CAPACITY];
    size_t count;
    size_t restarts;
    size_t overflows;
} capture_t;

static void feed(servo_frame_receiver_t *receiver, const char *data, size_t length, capture_t *capture)
{
    assert(receiver != NULL && data != NULL && capture != NULL);
    for (size_t index = 0U; index < length; ++index) {
        const servo_rx_event_t event =
            servo_frame_receiver_push(receiver, (uint8_t)data[index]);
        assert(event != SERVO_RX_INVALID_ARGUMENT);
        if (event == SERVO_RX_FRAME_READY) {
            assert(capture->count < TEST_CAPTURE_CAPACITY);
            assert(receiver->used <= SERVO_RECEIVER_MAX_FRAME_BYTES);
            assert(receiver->buffer[receiver->used] == '\0');
            capture->lengths[capture->count] = receiver->used;
            // 复制在下一次push前发生，避免保存指向会被覆盖的同一个缓冲区的指针。
            (void)memcpy(capture->frames[capture->count], receiver->buffer, receiver->used + 1U);
            capture->count += 1U;
        } else if (event == SERVO_RX_RESTARTED) {
            capture->restarts += 1U;
        } else if (event == SERVO_RX_OVERFLOW) {
            capture->overflows += 1U;
        }
    }
}

static void test_split_sticky_and_pending(void)
{
    const char sticky[] = "#002P1495!#002P1500!#002P1510!";
    const size_t length = sizeof(sticky) - 1U;
    for (size_t split = 0U; split <= length; ++split) {
        servo_frame_receiver_t receiver;
        capture_t capture = {0};
        const bool initialized = servo_frame_receiver_init(&receiver);
        assert(initialized);
        feed(&receiver, sticky, split, &capture);
        feed(&receiver, sticky + split, length - split, &capture);
        assert(capture.count == 3U && capture.restarts == 0U && capture.overflows == 0U);
        assert(strcmp(capture.frames[0], "#002P1495!") == 0);
        assert(strcmp(capture.frames[1], "#002P1500!") == 0);
        assert(strcmp(capture.frames[2], "#002P1510!") == 0);
    }

    servo_frame_receiver_t receiver;
    capture_t capture = {0};
    bool initialized = servo_frame_receiver_init(&receiver);
    assert(initialized);
    for (size_t index = 0U; index < length; ++index) {
        feed(&receiver, sticky + index, 1U, &capture);
    }
    assert(capture.count == 3U);

    initialized = servo_frame_receiver_init(&receiver);
    assert(initialized);
    capture = (capture_t){0};
    const char first[] = "#002P1495!#002P15";
    const char second[] = "00!";
    feed(&receiver, first, sizeof(first) - 1U, &capture);
    assert(capture.count == 1U && receiver.used == 7U && !receiver.frame_ready);
    assert(strcmp(receiver.buffer, "#002P15") == 0);
    feed(&receiver, second, sizeof(second) - 1U, &capture);
    assert(capture.count == 2U && strcmp(capture.frames[1], "#002P1500!") == 0);
}

static void test_noise_resynchronization_and_business_rejection(void)
{
    servo_frame_receiver_t receiver;
    capture_t capture = {0};
    const bool initialized = servo_frame_receiver_init(&receiver);
    assert(initialized);
    const char data[] = "noise!!#broken#002P1495!\r\n!##!#002P14A5!";
    feed(&receiver, data, sizeof(data) - 1U, &capture);
    assert(capture.count == 3U && capture.restarts == 2U);
    assert(strcmp(capture.frames[0], "#002P1495!") == 0);
    assert(strcmp(capture.frames[1], "#!") == 0);
    assert(strcmp(capture.frames[2], "#002P14A5!") == 0);
    uint16_t position = 0U;
    servo_codec_status_t status = servo_parse_position_reply(
        capture.frames[0], capture.lengths[0], 2U, &position);
    assert(status == SERVO_CODEC_OK && position == 1495U);
    status = servo_parse_position_reply(capture.frames[1], capture.lengths[1], 2U, &position);
    assert(status == SERVO_CODEC_LENGTH_ERROR && position == 1495U);
    status = servo_parse_position_reply(capture.frames[2], capture.lengths[2], 2U, &position);
    assert(status == SERVO_CODEC_FORMAT_ERROR && position == 1495U);
}

static void test_capacity_and_recovery(void)
{
    servo_frame_receiver_t receiver;
    capture_t capture = {0};
    bool initialized = servo_frame_receiver_init(&receiver);
    assert(initialized);
    char exact[SERVO_RECEIVER_MAX_FRAME_BYTES];
    (void)memset(exact, 'x', sizeof(exact));
    exact[0] = '#';
    exact[sizeof(exact) - 1U] = '!';
    feed(&receiver, exact, sizeof(exact), &capture);
    assert(capture.count == 1U && capture.lengths[0] == sizeof(exact));
    assert(capture.frames[0][sizeof(exact)] == '\0');

    initialized = servo_frame_receiver_init(&receiver);
    assert(initialized);
    capture = (capture_t){0};
    char oversized[SERVO_RECEIVER_MAX_FRAME_BYTES + 1U];
    (void)memset(oversized, 'x', sizeof(oversized));
    oversized[0] = '#';
    oversized[sizeof(oversized) - 1U] = '!';
    feed(&receiver, oversized, sizeof(oversized), &capture);
    assert(capture.count == 0U && capture.overflows == 1U && receiver.used == 0U);
    const char good[] = "#002P1495!";
    feed(&receiver, good, sizeof(good) - 1U, &capture);
    assert(capture.count == 1U && strcmp(capture.frames[0], good) == 0);

    // 旧半帧已达64字节时，新#优先重同步，不应误判为溢出后丢掉新帧头。
    initialized = servo_frame_receiver_init(&receiver);
    assert(initialized);
    capture = (capture_t){0};
    oversized[sizeof(oversized) - 1U] = 'x';
    feed(&receiver, oversized, SERVO_RECEIVER_MAX_FRAME_BYTES, &capture);
    feed(&receiver, good, sizeof(good) - 1U, &capture);
    assert(capture.count == 1U && capture.restarts == 1U && capture.overflows == 0U);
}

static void test_raw_bytes_and_independent_instances(void)
{
    servo_frame_receiver_t first;
    servo_frame_receiver_t second;
    capture_t first_capture = {0};
    capture_t second_capture = {0};
    bool initialized = servo_frame_receiver_init(&first);
    assert(initialized);
    initialized = servo_frame_receiver_init(&second);
    assert(initialized);
    const char raw[] = {'#', '0', '0', '2', 'P', '1', '\0', '9', '5', '!'};
    const char other[] = "#003P1500!";
    feed(&first, raw, 4U, &first_capture);
    feed(&second, other, sizeof(other) - 1U, &second_capture);
    feed(&first, raw + 4U, sizeof(raw) - 4U, &first_capture);
    assert(first_capture.count == 1U && second_capture.count == 1U);
    assert(first_capture.lengths[0] == sizeof(raw));
    assert(memcmp(first_capture.frames[0], raw, sizeof(raw)) == 0);
    assert(strcmp(second_capture.frames[0], other) == 0);
    uint16_t position = 1234U;
    servo_codec_status_t status = servo_parse_position_reply(
        first_capture.frames[0], first_capture.lengths[0], 2U, &position);
    assert(status == SERVO_CODEC_FORMAT_ERROR && position == 1234U);

    char high_byte[] = "#002P1495!";
    high_byte[6] = (char)0xFF;
    first_capture = (capture_t){0};
    feed(&first, high_byte, sizeof(high_byte) - 1U, &first_capture);
    status = servo_parse_position_reply(
        first_capture.frames[0], first_capture.lengths[0], 2U, &position);
    assert(status == SERVO_CODEC_FORMAT_ERROR && position == 1234U);
}

static void test_argument_errors_and_reset(void)
{
    bool initialized = servo_frame_receiver_init(NULL);
    assert(!initialized);
    servo_rx_event_t event = servo_frame_receiver_push(NULL, '#');
    assert(event == SERVO_RX_INVALID_ARGUMENT);
    servo_frame_receiver_t receiver;
    initialized = servo_frame_receiver_init(&receiver);
    assert(initialized);
    receiver.used = SERVO_RECEIVER_MAX_FRAME_BYTES + 1U;
    event = servo_frame_receiver_push(&receiver, '!');
    assert(event == SERVO_RX_INVALID_ARGUMENT);
    initialized = servo_frame_receiver_init(&receiver);
    assert(initialized && receiver.used == 0U && !receiver.frame_ready);
    event = servo_frame_receiver_push(&receiver, '#');
    assert(event == SERVO_RX_NONE && receiver.used == 1U);
    initialized = servo_frame_receiver_init(&receiver);
    assert(initialized && receiver.used == 0U && receiver.buffer[0] == '\0');
}

int main(void)
{
    test_split_sticky_and_pending();
    test_noise_resynchronization_and_business_rejection();
    test_capacity_and_recovery();
    test_raw_bytes_and_independent_instances();
    test_argument_errors_and_reset();
    return 0;
}
