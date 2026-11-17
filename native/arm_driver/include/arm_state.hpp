#ifndef JIXIEBI_ARM_STATE_HPP
#define JIXIEBI_ARM_STATE_HPP

#include "position_reader.hpp"
#include "servo_codec.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace jixiebi {

// 数量来自本项目协议编号0～5；六个舵机不代表已确认六个独立转动自由度。
inline constexpr std::size_t kServoCount = SERVO_ID_MAX + 1U;

enum class SampleStatus {
    NotQueried,  // 前面某轴失败，尚未向这个编号发送查询。
    Ok,         // 收到通过协议校验的匹配回复。
    Failed      // 已查询，但未获得可交付的位置。
};

struct ServoPositionSample {
    std::uint8_t servo_id = 0U;
    SampleStatus status = SampleStatus::NotQueried;
    // 只有status==Ok时位置才有效；0只是内存初值，不能显示成实物位置。
    std::uint16_t position_p = 0U;
    double elapsed_ms = 0.0;          // 本次read_position调用耗时，毫秒。
    double finished_offset_ms = 0.0;  // 调用结束距批次开始的毫秒数，不是设备采样时刻。
    std::string error;
};

struct ArmStateBatch {
    std::array<ServoPositionSample, kServoCount> samples{};
    std::size_t valid_count = 0U;
    bool complete = false;
    double elapsed_ms = 0.0;
};

// reader由main创建并持有：同一连接依次查询000～005，不在循环里另开串口。
// 每轴只查一次，第一处失败即结束；保留此前成功样本，后续标记NotQueried。
// timeout是每个编号的期限，不是整批期限；六轴来自先后时刻，不能当同步采样。
ArmStateBatch read_arm_state(PositionReader &reader, std::chrono::milliseconds timeout);

}  // namespace jixiebi

#endif  // JIXIEBI_ARM_STATE_HPP
