#ifndef JIXIEBI_JOINT_STATE_BRIDGE_CALIBRATED_FEEDBACK_HPP
#define JIXIEBI_JOINT_STATE_BRIDGE_CALIBRATED_FEEDBACK_HPP

#include "jixiebi_interfaces/msg/servo_states_raw.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace jixiebi {

using RawBatch = jixiebi_interfaces::msg::ServoStatesRaw;
using JointState = sensor_msgs::msg::JointState;

// 下标是本机已有的舵机地址；004末端滚转和005夹爪不属于第十二课四轴模型。
inline constexpr std::array<const char *, 4> kJointNames{
    "base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch"};

struct Calibration {
    int p_a;
    double angle_a_deg;
    int p_b;
    double angle_b_deg;
};

using Calibrations = std::array<std::optional<Calibration>, kJointNames.size()>;

// servo_id=-1 且端点留空返回 nullopt；填写不全或数据不合法则抛 invalid_argument。
std::optional<Calibration> checked_calibration(std::size_t joint, std::int64_t servo_id,
    std::int64_t p_a, double angle_a_deg, std::int64_t p_b, double angle_b_deg);

// 只在整批可用时替换 output。返回 nullptr 表示成功，其余为本批被拒绝的原因。
const char *convert_feedback(const RawBatch &raw, const Calibrations &calibrations,
                             JointState &output, std::optional<int> nominal_yaw_zero_p = std::nullopt);

}  // namespace jixiebi

#endif
