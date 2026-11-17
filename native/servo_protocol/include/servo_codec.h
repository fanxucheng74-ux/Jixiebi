#ifndef JIXIEBI_SERVO_CODEC_H
#define JIXIEBI_SERVO_CODEC_H

#include <stddef.h>  // size_t用于描述缓冲区容量，单位是字节。
#include <stdint.h>  // uint8_t用于保存本项目000～005的舵机编号。

// 这些常量描述查询帧的格式，不是关节行程或运动权限。
#define SERVO_ID_MAX (5U)
#define SERVO_POSITION_QUERY_LENGTH (9U)
#define SERVO_POSITION_QUERY_CAPACITY (SERVO_POSITION_QUERY_LENGTH + 1U)
// 回复#002P1495!是10个协议字节；这里的长度不包含C字符串的结尾\0。
#define SERVO_POSITION_REPLY_LENGTH (10U)
// 位置字段的协议数字范围，不代表机械限位、角度或已校准的实际坐标。
#define SERVO_POSITION_MIN_P (500U)
#define SERVO_POSITION_MAX_P (2500U)

// 调用者根据状态决定是否使用输出，不能把组帧失败的缓冲区继续发送。
typedef enum {
    SERVO_CODEC_OK = 0,
    SERVO_CODEC_INVALID_ARGUMENT,
    SERVO_CODEC_ID_OUT_OF_RANGE,
    SERVO_CODEC_BUFFER_TOO_SMALL,
    SERVO_CODEC_FORMAT_ERROR,
    // 新状态只追加在末尾，保持第一课已有状态的数值不变。
    SERVO_CODEC_LENGTH_ERROR,
    SERVO_CODEC_ID_MISMATCH,
    SERVO_CODEC_POSITION_OUT_OF_RANGE
} servo_codec_status_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 生成单个舵机的位置查询字符串，例如#004PRAD!；不访问串口。
 * @param servo_id 舵机编号，当前项目只允许0～5。外部输入须先校验再转换类型。
 * @param buffer 调用者提供的可写数组；实际可写容量必须至少为capacity字节。
 * @param capacity 数组容量，单位字节；成功至少需要10字节，包含结尾的\0。
 * @return 成功返回SERVO_CODEC_OK，否则返回具体错误码。
 * @note 非空buffer且capacity大于0时，失败输出为空字符串；容量0或空指针不写内存。
 *       成功时线上帧长度为9字节，末尾\0仅供C字符串使用，不属于舵机协议。
 */
servo_codec_status_t servo_make_position_query(uint8_t servo_id, char *buffer, size_t capacity);

/**
 * @brief 解析一条完整位置回包#dddPdddd!并匹配正在等待的舵机编号。
 * @param frame 可读的完整帧；无需以\0结尾，实际可读字节数必须至少为length。
 * @param length 实际帧长度，必须为10字节；不是接收缓冲区容量，不包含结尾\0。
 * @param expected_id 本次查询等待的编号0～5；外部输入须先校验再转换类型。
 * @param position_p 输出地址；全部检查通过后才写入协议P值，失败保持原值。
 * @return 成功返回SERVO_CODEC_OK；否则返回参数、长度、格式、编号或P范围错误。
 * @note 不读取串口、不拼接半包；没有请求序号，不能凭本函数区分同ID迟到回复。
 */
servo_codec_status_t servo_parse_position_reply(const char *frame, size_t length,
                                                uint8_t expected_id, uint16_t *position_p);

#ifdef __cplusplus
}
#endif

#endif  // JIXIEBI_SERVO_CODEC_H
