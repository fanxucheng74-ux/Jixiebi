#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <iostream>
#include <memory>

#include "jixiebi_interfaces/action/count_steps.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using CountSteps = jixiebi_interfaces::action::CountSteps;
using Client = rclcpp_action::Client<CountSteps>;
using Executor = rclcpp::executors::SingleThreadedExecutor;
using namespace std::chrono_literals;

// 在主循环中请求取消指定目标。取消答复只描述服务端是否受理；最终状态另从结果取。
void request_cancel(Executor &executor, Client &client, const Client::GoalHandle::SharedPtr &handle) {
    auto reply = client.async_cancel_goal(handle);
    std::cout << "CANCEL_REQUESTED" << std::endl;
    if (executor.spin_until_future_complete(reply, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        std::cerr << "CANCEL_ACK_UNKNOWN：未收到取消答复，继续等待任务结果\n";
        return;
    }
    const auto ack = reply.get();
    // 我们传入单个handle，核对取消清单中的目标UUID，确认受理的是同一任务。
    if (ack->return_code == Client::CancelResponse::ERROR_NONE &&
        ack->goals_canceling.size() == 1 &&
        ack->goals_canceling.front().goal_id.uuid == handle->get_goal_id()) {
        std::cout << "CANCEL_ACCEPTED" << std::endl;
    } else {
        std::cout << "CANCEL_NOT_ACCEPTED code=" << static_cast<int>(ack->return_code) << std::endl;
    }
}

int run_task(const rclcpp::Node::SharedPtr &node) {
    const auto steps = node->declare_parameter<std::int64_t>("steps", 6);
    const auto cancel_after = node->declare_parameter<std::int64_t>("cancel_after", 0);
    if (steps < 1 || steps > 20 || cancel_after < 0 || cancel_after >= steps) {
        std::cerr << "参数范围：1 <= steps <= 20，0 <= cancel_after < steps；0表示自然完成\n";
        return 1;
    }

    Executor executor;
    executor.add_node(node);
    auto client = rclcpp_action::create_client<CountSteps>(node, "/jixiebi/learning/count_steps");
    if (!client->wait_for_action_server(5s)) {
        std::cerr << "5秒内未发现Action服务端，目标未发送\n";
        return 1;
    }

    // 单线程执行器串行调度。反馈回调只保存步数，把发取消的操作留到回调结束后。
    std::uint32_t completed = 0;
    Client::SendGoalOptions options;
    options.feedback_callback =
        [&completed](Client::GoalHandle::SharedPtr, const std::shared_ptr<const CountSteps::Feedback> feedback) {
            completed = feedback->completed_steps;
            std::cout << "FEEDBACK=" << completed << std::endl;
        };
    CountSteps::Goal goal;
    goal.total_steps = static_cast<std::uint32_t>(steps);
    auto admission = client->async_send_goal(goal, options);  // 此处仅发送一次目标。
    if (executor.spin_until_future_complete(admission, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        std::cerr << "目标答复未知，未重发\n";
        return 1;
    }
    const auto handle = admission.get();  // 句柄关联这个目标的反馈、取消和最终结果。
    if (!handle) {
        std::cerr << "GOAL_REJECTED\n";
        return 1;
    }
    std::cout << "GOAL_ACCEPTED" << std::endl;

    auto result = client->async_get_result(handle);
    bool cancel_sent = false;
    // 每步500毫秒，最多20步。15秒含本机调度余量；每次spin只处理就绪事件。
    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (result.wait_for(0s) != std::future_status::ready) {
        if (!rclcpp::ok() || std::chrono::steady_clock::now() >= deadline) {
            std::cerr << "RESULT_UNKNOWN：结果等待中断或超时\n";
            return 1;
        }
        executor.spin_once(100ms);
        if (!cancel_sent && cancel_after > 0 && completed >= cancel_after &&
            result.wait_for(0s) != std::future_status::ready) {
            cancel_sent = true;  // 阈值后的每条反馈都可能继续到达，只请求取消一次。
            request_cancel(executor, *client, handle);
        }
    }

    // 同一目标可能先完成或先收到取消，按服务端给出的终态及对应载荷展示结果。
    const auto finished = result.get();
    using Code = rclcpp_action::ResultCode;
    const char *state = "UNKNOWN";
    switch (finished.code) {
        case Code::SUCCEEDED: state = "SUCCEEDED"; break;
        case Code::CANCELED: state = "CANCELED"; break;
        case Code::ABORTED: state = "ABORTED"; break;
        default: break;
    }
    std::cout << "RESULT=" << state << std::endl;
    if (!finished.result) {
        return 1;
    }
    std::cout << "COMPLETED_STEPS=" << finished.result->completed_steps << std::endl;
    const bool expected = cancel_after == 0 ? finished.code == Code::SUCCEEDED
                                             : cancel_sent && finished.code == Code::CANCELED;
    return expected ? 0 : 1;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int status = 1;
    try {
        status = run_task(std::make_shared<rclcpp::Node>("jixiebi_lesson08_client"));
    } catch (const std::exception &error) {
        std::cerr << "CLIENT_ERROR: " << error.what() << '\n';
    }
    rclcpp::shutdown();
    return status;
}
