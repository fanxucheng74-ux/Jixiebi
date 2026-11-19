#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_playback.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>

using Follow = control_msgs::action::FollowJointTrajectory;
using Client = rclcpp_action::Client<Follow>;
using namespace std::chrono_literals;

int run(const rclcpp::Node::SharedPtr &node)
{
    // 同一条 −10° 往返轨迹。0 表示自然完成，正数表示按反馈里的轨迹秒数取消。
    Follow::Goal goal;
    goal.trajectory = lesson_trajectory::make_preview(-10.0);
    const double end = lesson_trajectory::seconds_at(goal.trajectory.points.back());
    const double cancel_at = node->declare_parameter<double>("cancel_after_s", 0.0);
    if (!std::isfinite(cancel_at) || cancel_at < 0 || cancel_at >= end) {
        std::cerr << "cancel_after_s 须满足 0 ≤ t < 4 秒\n";
        return 1;
    }
    auto client = rclcpp_action::create_client<Follow>(node, "/jixiebi/learning/follow_joint_trajectory");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    if (!client->wait_for_action_server(5s)) {
        std::cerr << "未找到服务端，请先启动 WSL 终端 A\n";
        return 1;
    }

    double progress_s = 0.0;
    int last_bucket = -1;
    Client::SendGoalOptions options;
    options.feedback_callback = [&](Client::GoalHandle::SharedPtr, std::shared_ptr<const Follow::Feedback> fb) {
        progress_s = lesson_trajectory::seconds_at(fb->desired);
        // 服务端逐帧反馈，终端每半秒打印一次。本例发出的第三列为 elbow_flex。
        const int bucket = static_cast<int>(progress_s * 2);
        if (bucket != last_bucket && fb->desired.positions.size() > lesson_trajectory::kElbow &&
            fb->actual.positions.size() > lesson_trajectory::kElbow) {
            last_bucket = bucket;
            std::cout << std::fixed << std::setprecision(3) << "t=" << progress_s << "/" << end
                      << "s desired=" << fb->desired.positions[lesson_trajectory::kElbow]
                      << " rad model=" << fb->actual.positions[lesson_trajectory::kElbow] << " rad" << std::endl;
        }
    };
    // 目标只发送一次。接纳后的句柄标识本次任务，用它取消和获取终态。
    auto admission = client->async_send_goal(goal, options);
    if (executor.spin_until_future_complete(admission, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        std::cerr << "目标答复未知\n";
        return 1;
    }
    auto handle = admission.get();
    if (!handle) {
        std::cerr << "GOAL_REJECTED，请看 A 的拒绝原因\n";
        return 1;
    }
    std::cout << "GOAL_ACCEPTED" << std::endl;
    auto result = client->async_get_result(handle);
    bool cancel_sent = false;
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (result.wait_for(0s) != std::future_status::ready) {
        if (!rclcpp::ok() || std::chrono::steady_clock::now() > deadline) {
            std::cerr << "RESULT_UNKNOWN：未获得终态\n";
            return 1;
        }
        executor.spin_once(50ms);
        if (!cancel_sent && cancel_at > 0 && progress_s >= cancel_at &&
            result.wait_for(0s) != std::future_status::ready) {
            cancel_sent = true;
            // 在反馈回调返回后才等待取消答复，让执行器继续处理反馈与结果。
            auto ack = client->async_cancel_goal(handle);
            std::cout << "CANCEL_REQUESTED" << std::endl;
            if (executor.spin_until_future_complete(ack, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
                std::cout << "CANCEL_ACK_UNKNOWN" << std::endl;
            } else {
                const auto reply = ack.get();
                const bool accepted = reply->return_code == Client::CancelResponse::ERROR_NONE &&
                    reply->goals_canceling.size() == 1 &&
                    reply->goals_canceling.front().goal_id.uuid == handle->get_goal_id();
                std::cout << (accepted ? "CANCEL_ACCEPTED" : "CANCEL_NOT_ACCEPTED") << std::endl;
            }
        }
    }
    // FollowJointTrajectory 没有专门的取消错误码；用 Action 终态区分成功和取消。
    const auto done = result.get();
    using Code = rclcpp_action::ResultCode;
    if (done.code == Code::CANCELED) {
        std::cout << "RESULT=CANCELED" << std::endl;
        return cancel_sent ? 0 : 1;
    }
    if (done.code == Code::SUCCEEDED && done.result) {
        std::cout << "RESULT=SUCCEEDED error_code=" << done.result->error_code << std::endl;
        return done.result->error_code == Follow::Result::SUCCESSFUL && cancel_at == 0.0 ? 0 : 1;
    }
    std::cerr << (done.code == Code::ABORTED ? "RESULT=ABORTED" : "RESULT=UNKNOWN") << std::endl;
    if (done.result) std::cerr << done.result->error_string << std::endl;
    return 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int status = run(std::make_shared<rclcpp::Node>("lesson17_follow_client"));
    rclcpp::shutdown();
    return status;
}
