#pragma once

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

// 三课共用同一条例题和同一条插值曲线，避免消息入口不同导致模型动作不同。
namespace lesson_trajectory {
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using Point = trajectory_msgs::msg::JointTrajectoryPoint;

// 第 198 批的模型肘角为 0°，数组下标 2 对应 elbow_flex。
constexpr std::size_t kElbow = 2;
constexpr double kRadPerDegree = 3.14159265358979323846 / 180.0;
constexpr double kJointLimit = 90.0 * kRadPerDegree;  // 第十二课四个模型轴统一限位。

inline Trajectory make_preview(double elbow_offset_deg)
{
    Trajectory trajectory;
    // 第一步：确定每一列的关节名。每个点都沿用这个顺序，只有模型里的四个转轴。
    trajectory.joint_names = {"base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch"};

    // 第二步：复用第十四课第 198 批的标称角（度），在消息边界转成 ROS 使用的弧度。
    Point start;
    start.positions = {-0.135 * kRadPerDegree, 89.73 * kRadPerDegree, 0.0, 0.27 * kRadPerDegree};
    start.time_from_start.sec = 0;

    // 第三步：复制整个位姿，只改肘关节。2 表示从轨迹起点算起第 2 秒到达。
    Point bend = start;
    bend.positions[kElbow] += elbow_offset_deg * kRadPerDegree;
    bend.time_from_start.sec = 2;

    // 第四步：返回时沿用起点的关节角，把到达时刻设为从起点算起第 4 秒。
    Point back = start;
    back.time_from_start.sec = 4;
    trajectory.points = {start, bend, back};
    return trajectory;
}

// time_from_start 是从整条轨迹开始算的累计时间。保留纳秒，小数秒路点也能播放。
inline double seconds_at(const Point &point)
{
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
}

inline std::string check_preview(const Trajectory &path)
{
    // 第十二课的四个转轴都需要角度。列的顺序由发送者决定，同一名称只出现一次。
    const std::array<const char *, 4> joints{"base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch"};
    if (path.joint_names.size() != joints.size()) return "需要四个模型关节";
    for (const auto *name : joints) {
        if (std::count(path.joint_names.begin(), path.joint_names.end(), name) != 1) {
            return std::string("需要且只能有一列 ") + name;
        }
    }
    if (path.header.stamp.sec != 0 || path.header.stamp.nanosec != 0) return "本课收到消息即起播，请使用零时间戳";
    if (path.points.size() < 2 || seconds_at(path.points.front()) != 0.0) return "至少两个路点，首点时间为零";
    double previous = -1.0;
    for (const auto &point : path.points) {
        const double t = seconds_at(point);
        if (point.time_from_start.nanosec >= 1000000000u || t <= previous) return "路点时间须合法且严格递增";
        previous = t;
        if (point.positions.size() != joints.size()) return "每个路点需要四个角度";
        for (double q : point.positions) {
            // 四个转轴在本课 URDF 中均为 ±π/2，单位 rad。
            if (!std::isfinite(q) || std::abs(q) > kJointLimit) return "角度超出模型范围";
        }
        if (!point.velocities.empty() || !point.accelerations.empty() || !point.effort.empty()) {
            return "本课只解释位置路点，速度、加速度和力矩列应为空";
        }
    }
    return {};
}

// 给定播放秒数，计算此刻四个关节的位置。调用方已检查路点时间，分母一定大于零。
inline std::vector<double> positions_at(const Trajectory &path, double elapsed, bool sampled_linear = false)
{
    if (elapsed <= 0.0) return path.points.front().positions;
    if (elapsed >= seconds_at(path.points.back())) return path.points.back().positions;
    for (std::size_t i = 1; i < path.points.size(); ++i) {
        const auto &a = path.points[i - 1];
        const auto &b = path.points[i];
        if (elapsed > seconds_at(b)) continue;
        const double u = (elapsed - seconds_at(a)) / (seconds_at(b) - seconds_at(a));
        // 普通路点逐段起停；第 22 课的密集样本已含整条线的时间律，按时间线性连接。
        const double s = sampled_linear ? u : u * u * (3.0 - 2.0 * u);
        auto positions = a.positions;
        for (std::size_t j = 0; j < positions.size(); ++j) {
            positions[j] += s * (b.positions[j] - a.positions[j]);
        }
        return positions;
    }
    return path.points.back().positions;  // 计时器越过末点时精确保持终点。
}
}  // namespace lesson_trajectory
