#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "lesson_model_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_playback.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>

using Follow = control_msgs::action::FollowJointTrajectory;
using State = sensor_msgs::msg::JointState;
using namespace std::chrono_literals;

lesson_trajectory::Trajectory plan_from_state(const State &state, double target_deg)
{
    // 名字与位置按列配对。确定 elbow_flex 在本帧的哪一列，不能假设它总在第三列。
    const auto elbow = std::find(state.name.begin(), state.name.end(), "elbow_flex");
    if (elbow == state.name.end() || state.position.size() != state.name.size()) {
        throw std::invalid_argument("状态缺少肘关节或关节名与角度数量不符");
    }
    lesson_trajectory::Trajectory path;
    path.joint_names = state.name;
    lesson_trajectory::Point start;
    start.positions = state.position;  // 刚收到的四个模型角，单位 rad；首点时间默认 0。
    auto finish = start;               // 复制整帧，另外三轴保持原位。
    const auto column = std::distance(state.name.begin(), elbow);
    finish.positions[column] = target_deg * lesson_trajectory::kRadPerDegree;
    finish.time_from_start.sec = 2;    // 从新起点出发，两秒到绝对目标角。
    path.points = {start, finish};
    const auto reason = lesson_trajectory::check_preview(path);
    if (!reason.empty()) throw std::invalid_argument(reason);
    std::cout << std::fixed << std::setprecision(3)
              << "START_DEG=" << start.positions[column] / lesson_trajectory::kRadPerDegree
              << " TARGET_DEG=" << target_deg << std::endl;
    return path;
}

int run(const rclcpp::Node::SharedPtr &node)
{
    const double target = node->declare_parameter<double>("elbow_target_deg", -10.0);
    if (!std::isfinite(target) || std::abs(target) > 90.0) {
        throw std::invalid_argument("elbow_target_deg 须在本课模型的 [-90, 90] 度范围内");
    }
    auto client = rclcpp_action::create_client<Follow>(node, "/jixiebi/learning/follow_joint_trajectory");
    if (!client->wait_for_action_server(5s)) throw std::runtime_error("未发现模型 Action，请先启动终端 A");

    // 先等服务端就绪，再临时订阅一帧；这样可直接在第十七课取消后继续。
    State state;
    if (!rclcpp::wait_for_message(state, node, "/jixiebi/learning/follow/joint_states", 3s)) {
        throw std::runtime_error("3 秒内没有收到模型状态");
    }
    const auto age = node->now() - rclcpp::Time(state.header.stamp);
    if (age.nanoseconds() < 0 || age.seconds() > 0.5) {
        throw std::runtime_error("模型时间戳超前或样本超过 0.5 秒");
    }
    Follow::Goal goal;
    goal.trajectory = plan_from_state(state, target);
    if (goal.trajectory.points.front().positions == goal.trajectory.points.back().positions) {
        std::cout << "ALREADY_AT_TARGET：采样时已在目标，本次未发送轨迹" << std::endl;
        return 0;
    }

    // 复用第十七课的 Action。客户端只规划一次；服务端负责起点核对与定时插值。
    lesson_action::play(node, client, goal);
    return 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int status = 1;
    try {
        status = run(std::make_shared<rclcpp::Node>("lesson18_from_state"));
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
    }
    rclcpp::shutdown();
    return status;
}
