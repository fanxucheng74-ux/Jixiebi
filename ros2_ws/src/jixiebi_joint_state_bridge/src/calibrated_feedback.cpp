#include "calibrated_feedback.hpp"
#include "nominal_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace jixiebi {
namespace {

// 仅用于识别协议可表示的 P；每个关节自身的可显示区间由它的两个实测点给出。
bool valid_protocol_p(std::int64_t p)
{
    return p >= 500 && p <= 2500;
}

const char *check_batch(const RawBatch &raw)
{
    if (raw.source != "SERIAL_TTY") {
        return "来源不是 SERIAL_TTY";
    }
    if (!raw.complete || raw.simultaneous || raw.valid_count != raw.servos.size()) {
        return "需要第六课完整的六轴顺序查询";
    }
    if (raw.sequence == 0 || raw.stamp.sec < 0 || raw.stamp.nanosec >= 1000000000U ||
        (raw.stamp.sec == 0 && raw.stamp.nanosec == 0)) {
        return "RK 批次序号或时间戳无效";
    }
    if (!std::isfinite(raw.elapsed_ms) || raw.elapsed_ms < 0.0) {
        return "批次耗时无效";
    }
    double previous_offset_ms = 0.0;
    for (std::size_t id = 0; id < raw.servos.size(); ++id) {
        const auto &sample = raw.servos[id];
        if (sample.servo_id != id || sample.status != jixiebi_interfaces::msg::ServoStateRaw::OK ||
            !sample.error.empty() || !valid_protocol_p(sample.position_p)) {
            return "轴地址、状态、错误字段或协议 P 异常";
        }
        // 偏移与耗时都来自 RK 单调钟；用同批毫秒量互检，不与 WSL 的时钟比较。
        if (!std::isfinite(sample.elapsed_ms) || !std::isfinite(sample.finished_offset_ms) ||
            sample.elapsed_ms < 0.0 || sample.elapsed_ms > sample.finished_offset_ms ||
            sample.finished_offset_ms < previous_offset_ms || sample.finished_offset_ms > raw.elapsed_ms) {
            return "轴耗时或顺序查询的完成偏移无效";
        }
        previous_offset_ms = sample.finished_offset_ms;
    }
    return nullptr;
}

}  // namespace

std::optional<Calibration> checked_calibration(std::size_t joint, std::int64_t servo_id,
    std::int64_t p_a, double angle_a_deg, std::int64_t p_b, double angle_b_deg)
{
    const std::string name = kJointNames.at(joint);
    if (servo_id == -1 && p_a == -1 && p_b == -1 &&
        std::isnan(angle_a_deg) && std::isnan(angle_b_deg)) {
        return std::nullopt;
    }
    if (servo_id != static_cast<std::int64_t>(joint)) {
        throw std::invalid_argument(name + " 的 servo_id 应为 " + std::to_string(joint));
    }
    // 至少留一个可插值的整数 P。此处只判数据可用性，跨度足否盖过噪声还需外测。
    if (!valid_protocol_p(p_a) || !valid_protocol_p(p_b) || std::abs(p_b - p_a) < 2) {
        throw std::invalid_argument(name + " 的两个实读 P 须在 [500, 2500] 且至少相差 2 P");
    }
    // ±90° 来自第十二课 URDF 的显示范围。它不是电机行程，也不授权运动。
    if (!std::isfinite(angle_a_deg) || !std::isfinite(angle_b_deg) ||
        std::abs(angle_a_deg) > 90.0 || std::abs(angle_b_deg) > 90.0 || angle_a_deg == angle_b_deg) {
        throw std::invalid_argument(name + " 的两个有符号实测角须不同，且在模型 [-90, 90] 度内");
    }
    return Calibration{static_cast<int>(p_a), angle_a_deg, static_cast<int>(p_b), angle_b_deg};
}

const char *convert_feedback(const RawBatch &raw, const Calibrations &calibrations,
                             JointState &output, std::optional<int> nominal_yaw_zero_p)
{
    if (const char *error = check_batch(raw)) {
        return error;
    }
    JointState next;
    next.header.stamp = raw.stamp;  // 整批顺序查询结束时的 RK 主机时间。
    constexpr double kRadPerDegree = 3.14159265358979323846 / 180.0;
    for (std::size_t id = 0; id < calibrations.size(); ++id) {
        const int p = raw.servos[id].position_p;
        double angle_deg;
        if (nominal_yaw_zero_p) {
            // 复用旧 URDF 的四轴有符号斜率。现场选的工作零点只平移底座，不改变肩轴 90° 的参考。
            const int zero = id == 0 ? *nominal_yaw_zero_p : kVendorReferenceP;
            angle_deg = kVendorReferenceDeg[id] + (p - zero) * kVendorDegreesPerP[id];
        } else {
            if (!calibrations[id]) {
                continue;  // 实测模式缺标定就省略该关节。
            }
            const auto &c = *calibrations[id];
            if (p < std::min(c.p_a, c.p_b) || p > std::max(c.p_a, c.p_b)) {
                return "已标定关节越出两个实测 P 的区间";
            }
            // 两个实测端点保留方向，仅在区间内插值。
            const double fraction = static_cast<double>(p - c.p_a) / (c.p_b - c.p_a);
            angle_deg = c.angle_a_deg + fraction * (c.angle_b_deg - c.angle_a_deg);
        }
        next.name.emplace_back(kJointNames[id]);
        next.position.push_back(angle_deg * kRadPerDegree);
    }
    if (next.name.empty()) {
        return "没有已标定关节";
    }
    output = std::move(next);  // 速度和力矩未测，两个数组均留空。
    return nullptr;
}

}  // namespace jixiebi
