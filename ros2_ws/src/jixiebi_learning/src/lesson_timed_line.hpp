#pragma once

#include "lesson_line_run.hpp"

namespace lesson_line {
// 每段的终点都采到，起点由上一段提供。采样间隔由调用课程给定，时间单位 s。
template<class Sample>
void sample_interval(double begin, double end, double step_s, Sample sample)
{
    const int n = static_cast<int>(std::ceil((end - begin) / step_s));
    for (int i = 1; i <= n; ++i) sample(begin + (end - begin) * i / n);
}

template<class Profile>
Trajectory make_timed_line(const KDL::Chain &chain, const KDL::JntArray &current,
                           const Profile &profile, double &entry_s, double step_s = 0.05)
{
    const auto geometry = lesson_ik::read_geometry(chain);
    auto seed = current;
    Trajectory path;
    path.joint_names.assign(lesson_ik::names.begin(), lesson_ik::names.end());
    auto append = [&](const lesson_ik::Angles &q, double t) {
        Point point;
        point.positions.assign(q.begin(), q.end());
        point.time_from_start = rclcpp::Duration::from_seconds(t);
        path.points.push_back(point);
        for (size_t j = 0; j < q.size(); ++j) seed(j) = q[j];
    };

    // 先从当前模型角平滑进场到左端。使用上一课的逆解和模型 1 rad/s 关节速度上限。
    lesson_ik::Angles start;
    for (size_t j = 0; j < start.size(); ++j) start[j] = current(j);
    append(start, 0.0);
    const auto first = lesson_ik::solve(geometry, left, pitch, seed);
    lesson_ik::check_solution(chain, first, left, pitch);
    double delta = 0.0;
    for (size_t j = 0; j < first.size(); ++j) delta = std::max(delta, std::abs(first[j] - start[j]));
    if (delta > 1e-8) {
        entry_s = std::max(1.0, 1.5 * delta);
        sample_interval(0.0, entry_s, step_s, [&](double t) {
            const double u = t / entry_s;
            const double s = u*u*(3.0 - 2.0*u);
            auto q = start;
            for (size_t j = 0; j < q.size(); ++j) q[j] += (first[j] - start[j]) * s;
            append(q, t);
        });
    }
    // 时间决定已经走过的毫米数，再把同一条直线的坐标交给已有逆解。
    auto line_sample = [&](double t) {
        const double u = profile.distance_mm(t) / profile.length_mm;
        const auto xyz = left + (right - left) * u;
        const auto q = lesson_ik::solve(geometry, xyz, pitch, seed);
        lesson_ik::check_solution(chain, q, xyz, pitch);
        append(q, entry_s + t);
    };
    const double brake_start = profile.duration() - Profile::ramp_s;
    sample_interval(0.0, Profile::ramp_s, step_s, line_sample);
    sample_interval(Profile::ramp_s, brake_start, step_s, line_sample);
    sample_interval(brake_start, profile.duration(), step_s, line_sample);
    const auto reason = lesson_trajectory::check_preview(path);
    if (!reason.empty()) throw std::runtime_error(reason);
    return path;
}
}  // namespace lesson_line
