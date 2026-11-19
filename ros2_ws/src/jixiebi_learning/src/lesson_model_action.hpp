#pragma once

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace lesson_action {
using Follow = control_msgs::action::FollowJointTrajectory;
using namespace std::chrono_literals;

// 两点与多点轨迹共用接单和终态判断；画轨迹时可传入反馈回调。
inline void play(const rclcpp::Node::SharedPtr &node,
                 const rclcpp_action::Client<Follow>::SharedPtr &client, const Follow::Goal &goal,
                 const rclcpp_action::Client<Follow>::SendGoalOptions &options = {})
{
    auto admission = client->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(node, admission, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("目标答复未知，未重发");
    }
    auto handle = admission.get();
    if (!handle) throw std::runtime_error("GOAL_REJECTED，请看 A 的起点或占用提示");
    auto result = client->async_get_result(handle);
    // 按末点时长给多点轨迹预留 2 秒答复时间，保留原先至少 6 秒的等待。
    const auto duration = rclcpp::Duration(goal.trajectory.points.back().time_from_start).to_chrono<std::chrono::nanoseconds>();
    const auto timeout = std::max(std::chrono::nanoseconds(6s), duration + 2s);
    if (rclcpp::spin_until_future_complete(node, result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("RESULT_UNKNOWN：未获得终态");
    }
    const auto done = result.get();
    if (done.code != rclcpp_action::ResultCode::SUCCEEDED || !done.result ||
        done.result->error_code != Follow::Result::SUCCESSFUL) {
        throw std::runtime_error("轨迹未成功完成");
    }
    std::cout << "RESULT=SUCCEEDED" << std::endl;
}
}  // namespace lesson_action
