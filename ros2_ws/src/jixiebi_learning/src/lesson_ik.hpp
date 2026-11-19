#pragma once

#include "trajectory_playback.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"

#include <array>
#include <limits>
#include <stdexcept>

// 第二十、二十一课共用同一套几何、分支选择和 URDF 回算。
namespace lesson_ik {
using Angles = std::array<double, 4>;
using lesson_trajectory::kJointLimit;
using lesson_trajectory::kRadPerDegree;
constexpr double kPi = 180.0 * kRadPerDegree;
inline const std::array<std::string, 4> names{"base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch"};

struct Geometry {
    double height, upper, forearm, tool;  // 单位 m。肩轴高度从 base_link 原点起算。
};

inline Geometry read_geometry(const KDL::Chain &chain)
{
    // 本课的解析式专用于第十二课的四转轴和平行俯仰机构。杆长从运行中的 URDF 取。
    if (chain.getNrOfSegments() != 5 || chain.getNrOfJoints() != 4) {
        throw std::runtime_error("需要第十二课的四轴加固定工具段模型");
    }
    for (size_t i = 0; i < names.size(); ++i) {
        if (chain.getSegment(i).getJoint().getName() != names[i]) throw std::runtime_error("运动链关节顺序不符");
    }
    return {chain.getSegment(0).pose(0).p.z(), chain.getSegment(2).pose(0).p.y(),
            chain.getSegment(3).pose(0).p.y(), chain.getSegment(4).pose(0).p.y()};
}

inline double wrap(double rad) { return std::remainder(rad, 2.0 * kPi); }

inline Angles solve(const Geometry &g, const KDL::Vector &target, double pitch, const KDL::JntArray &current)
{
    const double radius = std::hypot(target.x(), target.y());
    // 零姿沿 +Y；绕 +Z 正转后投向 −X。轴心上方的 XYZ 无法确定底座朝向，沿用当前角。
    const double yaw = radius > 1e-12 ? std::atan2(-target.x(), target.y()) : current(0);
    double best_cost = std::numeric_limits<double>::infinity();
    Angles best{};
    for (int side : {1, -1}) {
        // 同一目标也可能落在转动平面的反向延长线上。轴心处两支重合。
        if (radius <= 1e-12 && side == -1) continue;
        const double base = side == 1 ? yaw : wrap(yaw + kPi);
        const double r = side * radius - g.tool * std::cos(pitch);
        const double z = target.z() - g.height - g.tool * std::sin(pitch);
        const double cos_e = (r*r + z*z - g.upper*g.upper - g.forearm*g.forearm)
                             / (2.0 * g.upper * g.forearm);
        // 三角形不存在就排除这一支。clamp 只消除可达边界的浮点舍入。
        if (!std::isfinite(cos_e) || std::abs(cos_e) > 1.0 + 1e-12) continue;
        const double bend = std::acos(std::clamp(cos_e, -1.0, 1.0));
        for (double elbow : {-bend, bend}) {
            const double shoulder = wrap(std::atan2(z, r) -
                std::atan2(g.forearm * std::sin(elbow), g.upper + g.forearm * std::cos(elbow)));
            Angles q{base, shoulder, elbow, wrap(pitch - shoulder - elbow)};
            double cost = 0.0;
            for (size_t i = 0; i < q.size(); ++i) {
                if (std::abs(q[i]) > kJointLimit + 1e-12) {
                    cost = std::numeric_limits<double>::infinity();
                    break;
                }
                q[i] = std::clamp(q[i], -kJointLimit, kJointLimit);
                const double change = q[i] - current(i);
                cost += change * change;  // 从当前模型姿态出发，优先总关节变化较小的解。
            }
            if (cost < best_cost) { best = q; best_cost = cost; }
        }
    }
    if (!std::isfinite(best_cost)) throw std::runtime_error("NO_IK_SOLUTION：该位置与俯仰组合在模型限位内无解");
    return best;
}

inline double check_solution(const KDL::Chain &chain, const Angles &q,
                             const KDL::Vector &target, double pitch)
{
    KDL::JntArray joints(q.size());
    for (size_t i = 0; i < q.size(); ++i) joints(i) = q[i];
    KDL::Frame tip;
    KDL::ChainFkSolverPos_recursive fk(chain);
    if (fk.JntToCart(joints, tip) < 0) throw std::runtime_error("KDL 正运动学失败");
    const KDL::Vector direction(-std::sin(q[0])*std::cos(pitch), std::cos(q[0])*std::cos(pitch), std::sin(pitch));
    const double error_mm = (tip.p - target).Norm() * 1000.0;
    if (error_mm > 1e-4 || (tip.M.UnitY() - direction).Norm() > 1e-8) {
        throw std::runtime_error("解析解与运行中的 URDF 回算不符");
    }
    return error_mm;
}
}  // namespace lesson_ik
