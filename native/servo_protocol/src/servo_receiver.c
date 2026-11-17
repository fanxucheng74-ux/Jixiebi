#include "servo_receiver.h"

bool servo_frame_receiver_init(servo_frame_receiver_t *receiver)
{
    if (receiver == NULL) {
        return false;
    }
    receiver->used = 0U;
    receiver->buffer[0] = '\0';
    receiver->frame_ready = false;
    return true;
}

servo_rx_event_t servo_frame_receiver_push(servo_frame_receiver_t *receiver, uint8_t byte)
{
    // 防止空指针和部分非法状态导致越界；这不能代替调用前正确初始化。
    if (receiver == NULL || receiver->used > SERVO_RECEIVER_MAX_FRAME_BYTES) {
        return SERVO_RX_INVALID_ARGUMENT;
    }

    // 上一帧已交付，本次开始接下一字节；调用者应在此前处理/复制完整帧。
    if (receiver->frame_ready) {
        receiver->used = 0U;
        receiver->buffer[0] = '\0';
        receiver->frame_ready = false;
    }

    // 新#优先于溢出检查，可从尚未结束或已占满的坏半帧立即重新同步。
    if (byte == '#') {
        const bool restarted = receiver->used > 0U;
        receiver->buffer[0] = '#';
        receiver->buffer[1] = '\0';
        receiver->used = 1U;
        if (restarted) {
            return SERVO_RX_RESTARTED;
        }
        return SERVO_RX_NONE;
    }

    // 未见帧头时丢弃噪声，包括多余的!；不让噪声占用接收缓冲区。
    if (receiver->used == 0U) {
        return SERVO_RX_NONE;
    }

    // 一定先查容量再写入。溢出时丢弃整段，余下字节须等新#才重新收集。
    if (receiver->used >= SERVO_RECEIVER_MAX_FRAME_BYTES) {
        receiver->used = 0U;
        receiver->buffer[0] = '\0';
        receiver->frame_ready = false;
        return SERVO_RX_OVERFLOW;
    }

    receiver->buffer[receiver->used] = (char)byte;
    receiver->used += 1U;
    // 这个\0方便显示；used仍按实际收到的字节累计，不依赖字符串长度。
    receiver->buffer[receiver->used] = '\0';

    if (byte == '!') {
        receiver->frame_ready = true;
        return SERVO_RX_FRAME_READY;
    }
    return SERVO_RX_NONE;
}
