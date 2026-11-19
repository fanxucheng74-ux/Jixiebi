#pragma once

#include "kdl/frames.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace lesson_speed {
struct Profile {
    double length_mm;
    double cruise_mm_s;
    static constexpr double ramp_s = 0.5;  // 起步和刹车各占半秒，中央留给巡航。

    double duration() const { return length_mm / cruise_mm_s + ramp_s; }

    double distance_mm(double t) const
    {
        const double total = duration();
        t = std::clamp(t, 0.0, total);
        const double a = cruise_mm_s / ramp_s;
        if (t < ramp_s) return 0.5 * a * t * t;
        if (t <= total - ramp_s) return cruise_mm_s * (t - 0.5 * ramp_s);
        const double remaining = total - t;
        return length_mm - 0.5 * a * remaining * remaining;
    }
};

// 速度取自相邻两个显示帧的 FK 位置和反馈时间；参考速度另用时间律计算。
class SpeedMeter {
public:
    explicit SpeedMeter(Profile profile) : profile_(profile) {}

    void sample(double t, const KDL::Vector &tip)
    {
        if (!has_sample_) {
            has_sample_ = true;
            first_t_ = previous_t_ = t;
            previous_tip_ = tip;
            return;
        }
        const double dt = t - previous_t_;
        if (dt < 1e-6) return;  // 同时刻的反馈不能用来做除法。
        const auto d = tip - previous_tip_;
        const double ds_mm = 1000.0 * std::hypot(d.x(), d.y(), d.z());
        const double speed = ds_mm / dt;
        const double reference = (profile_.distance_mm(t) - profile_.distance_mm(previous_t_)) / dt;
        const double brake_start = profile_.duration() - Profile::ramp_s;
        // 只统计两个端点都处在巡航区间的速度，避免把跨越加速边界的一帧混入。
        if (previous_t_ >= Profile::ramp_s && t <= brake_start) {
            cruise_min_ = std::min(cruise_min_, speed);
            cruise_max_ = std::max(cruise_max_, speed);
            cruise_distance_ += ds_mm;
            cruise_time_ += dt;
        }
        if (t >= next_print_ || t >= profile_.duration() - 1e-6) {
            const char *phase = t < Profile::ramp_s ? "ACCEL" : (t <= brake_start ? "CRUISE" : "BRAKE");
            std::cout << std::fixed << std::setprecision(3) << "t=" << t << "s " << phase
                      << " REF_MM_S=" << reference << " MODEL_MM_S=" << speed << '\n';
            next_print_ = t + 0.22;
        }
        distance_ += ds_mm;
        previous_t_ = t;
        previous_tip_ = tip;
    }

    void report() const
    {
        if (cruise_time_ <= 0) throw std::runtime_error("缺少巡航区间的反馈，无法测量速度");
        const double observed_s = previous_t_ - first_t_;
        std::cout << std::fixed << std::setprecision(3)
                  << "CRUISE_MIN_MM_S=" << cruise_min_ << " CRUISE_MEAN_MM_S=" << cruise_distance_ / cruise_time_
                  << " CRUISE_MAX_MM_S=" << cruise_max_ << "\nOBSERVED_LINE_S=" << observed_s
                  << " OBSERVED_AVG_MM_S=" << distance_ / observed_s << std::endl;
    }

private:
    Profile profile_;
    KDL::Vector previous_tip_;
    bool has_sample_ = false;
    double first_t_ = 0, previous_t_ = 0, next_print_ = 0;
    double cruise_min_ = std::numeric_limits<double>::infinity(), cruise_max_ = 0;
    double cruise_distance_ = 0, cruise_time_ = 0, distance_ = 0;
};
}  // namespace lesson_speed
