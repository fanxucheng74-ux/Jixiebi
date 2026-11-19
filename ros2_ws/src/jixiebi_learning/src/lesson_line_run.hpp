#pragma once

#include "lesson_ik.hpp"
#include "lesson_model.hpp"
#include "lesson_model_action.hpp"
#include "lesson21_path_view.hpp"

#include <functional>
#include <iomanip>

namespace lesson_line {
using lesson_action::Follow;
using lesson_trajectory::Point;
using lesson_trajectory::Trajectory;
using namespace std::chrono_literals;

// 第十二课模型。端点在 base_link 中，以 m 表示；工具俯仰相对转动平面的水平正向。
inline const KDL::Vector left(-0.070, 0.180, 0.445), right(0.070, 0.180, 0.445);
constexpr double pitch = 55.0 * lesson_trajectory::kRadPerDegree;

// 共用同一份反馈和球心标记；各课只提供自己的路点生成与可选的速度观察函数。
using PlanLine = std::function<Trajectory(const KDL::Chain &, const KDL::JntArray &, double &)>;
using ObserveTip = std::function<void(double, const KDL::Vector &)>;
inline void run_model_line(const rclcpp::Node::SharedPtr &node, const std::string &action_name,
                           const PlanLine &plan, const ObserveTip &on_sample = {})
{
    const auto chain = lesson_model::read_chain(node);
    auto client = rclcpp_action::create_client<Follow>(node, action_name);
    if (!client->wait_for_action_server(5s)) throw std::runtime_error("未发现模型播放器");
    const auto current = lesson_model::angles_in_chain_order(chain, lesson_model::read_state(node));
    Follow::Goal goal;
    double entry_s = 0.0;
    goal.trajectory = plan(chain, current, entry_s);
    std::cout << "ENTRY_END_S=" << entry_s << std::endl;

    // actual 是播放器刚显示的模型角，用 KDL 还原黄色球心。只订阅本次目标的反馈。
    KDL::ChainFkSolverPos_recursive fk(chain);
    lesson_line::PathView view(node, left, right);
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.feedback_callback = [&](auto, std::shared_ptr<const Follow::Feedback> fb) {
        if (lesson_trajectory::seconds_at(fb->actual) < entry_s) return;  // 进场段不计入横移。
        if (fb->joint_names != goal.trajectory.joint_names || fb->actual.positions.size() != 4) {
            throw std::runtime_error("模型反馈的关节列与本次轨迹不符");
        }
        KDL::JntArray q(4);
        for (unsigned j = 0; j < q.rows(); ++j) q(j) = fb->actual.positions[j];
        KDL::Frame tip;
        if (fk.JntToCart(q, tip) < 0) throw std::runtime_error("反馈正运动学失败");
        view.observe(tip.p);
        if (on_sample) on_sample(lesson_trajectory::seconds_at(fb->actual) - entry_s, tip.p);
    };
    lesson_action::play(node, client, goal, options);
    if (view.samples() < 2) throw std::runtime_error("反馈采样不足，无法验证横移轨迹");
    // Action 完成后取一帧新的 JointState；终点误差与沿途最大偏差分别统计。
    const auto final_q = lesson_model::angles_in_chain_order(chain, lesson_model::read_state(node));
    KDL::Frame end;
    if (fk.JntToCart(final_q, end) < 0) throw std::runtime_error("终点正运动学失败");
    std::cout << std::fixed << std::setprecision(3)
              << "SAMPLES=" << view.samples() << " MAX_LINE_ERROR_MM=" << view.max_error_mm
              << " END_ERROR_MM=" << (end.p - right).Norm() * 1000.0 << std::endl;
}
}  // namespace lesson_line
