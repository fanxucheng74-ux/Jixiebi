#pragma once

#include "lesson22_speed_profile.hpp"

namespace lesson_ramp {
enum class Shape { Trapezoid, Smooth };

// 继承第 22 课的路径长度（mm）、巡航速度（mm/s）、起停时间 R 和横移总时长 T。
struct RampProfile : lesson_speed::Profile {
    Shape shape;

    // u 是起步经过时间占 R 的比例。对 v/V = 3u²−2u³ 积分，得到实际走过的毫米数。
    double ramp_distance_mm(double u) const
    {
        return cruise_mm_s * ramp_s * u * u * u * (1.0 - 0.5 * u);
    }

    double distance_mm(double t) const
    {
        if (shape == Shape::Trapezoid) return lesson_speed::Profile::distance_mm(t);
        t = std::clamp(t, 0.0, duration());
        if (t < ramp_s) return ramp_distance_mm(t / ramp_s);
        if (t <= duration() - ramp_s) return cruise_mm_s * (t - ramp_s / 2.0);
        // 刹车与起步对称：用剩余时间算剩余距离，保证终点恰好是全长。
        return length_mm - ramp_distance_mm((duration() - t) / ramp_s);
    }

    double speed_mm_s(double t) const
    {
        const double u = std::clamp(std::min(t, duration() - t) / ramp_s, 0.0, 1.0);
        if (shape == Shape::Trapezoid) return cruise_mm_s * u;
        return cruise_mm_s * u * u * (3.0 - 2.0 * u);
    }

    double accel_mm_s2(double t) const
    {
        // 梯形的加速度在切换处跳变；文件记录该时刻右侧的值。
        if (t < 0.0 || t >= duration()) return 0.0;
        const bool braking = t >= duration() - ramp_s;
        if (!braking && t >= ramp_s) return 0.0;
        const double u = braking ? (duration() - t) / ramp_s : t / ramp_s;
        const double scale = shape == Shape::Smooth ? 6.0 * u * (1.0 - u) : 1.0;
        return (braking ? -1.0 : 1.0) * cruise_mm_s / ramp_s * scale;
    }
};
}  // namespace lesson_ramp
