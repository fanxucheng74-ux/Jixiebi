#ifndef JIXIEBI_SERVO_RECEIVER_H
#define JIXIEBI_SERVO_RECEIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// 本项目的内存策略：一帧最多64字节，含#和!；不是厂家声明的协议上限。
#define SERVO_RECEIVER_MAX_FRAME_BYTES (64U)
#define SERVO_RECEIVER_BUFFER_CAPACITY (SERVO_RECEIVER_MAX_FRAME_BYTES + 1U)

typedef struct {
    char buffer[SERVO_RECEIVER_BUFFER_CAPACITY];  // 收集到的字节，末尾额外保留\0。
    size_t used;                                // 实际协议字节数，不包含末尾\0。
    bool frame_ready;                           // 刚收到了!，当前帧可供调用者处理。
} servo_frame_receiver_t;

// 这些是分帧事件；FRAME_READY不等于位置回复格式正确或真实状态新鲜。
typedef enum {
    SERVO_RX_NONE = 0,
    SERVO_RX_FRAME_READY,
    SERVO_RX_RESTARTED,
    SERVO_RX_OVERFLOW,
    SERVO_RX_INVALID_ARGUMENT
} servo_rx_event_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化或明确丢弃当前半帧；仅启动/显式复位时调用，不在每个chunk前调用。
 * @param receiver 调用者持有的接收状态，非空时清空有效长度和就绪标记。
 * @return 初始化成功返回true；空指针返回false。
 */
bool servo_frame_receiver_init(servo_frame_receiver_t *receiver);

/**
 * @brief 向已初始化的接收器送入一个字节；不连接串口、不校验位置业务内容。
 * @param receiver 已初始化且由单一调用者使用的接收器，不提供线程同步。
 * @param byte 本次收到的原始字节，包括可能的0字节；不能用strlen代替接收长度。
 * @return 返回分帧事件；OVERFLOW丢弃超长片段，RESTARTED表示新#替换旧半帧。
 * @note FRAME_READY时，buffer及used仅在下一次push/init之前保持可用；调用者
 *       必须立即处理或复制，不得等喂完整个chunk后才只读取最后一帧。
 *       无内部帧队列；新#总会重新同步，帧外噪声忽略。未初始化对象不得调用。
 */
servo_rx_event_t servo_frame_receiver_push(servo_frame_receiver_t *receiver, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif  // JIXIEBI_SERVO_RECEIVER_H
