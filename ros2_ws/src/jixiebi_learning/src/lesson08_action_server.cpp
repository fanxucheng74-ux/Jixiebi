#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>

#include "jixiebi_interfaces/action/count_steps.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using CountSteps = jixiebi_interfaces::action::CountSteps;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<CountSteps>;
using namespace std::chrono_literals;

class CountStepsServer : public rclcpp::Node {
public:
    CountStepsServer() : Node("jixiebi_lesson08_server") {
        // 每次定时回调只完成一步，随即把线程交还给spin处理目标与取消请求。
        timer_ = create_wall_timer(500ms, [this]() { advance_one_step(); });
        timer_->cancel();  // 收到第一个有效目标后才开始计时。

        server_ = rclcpp_action::create_server<CountSteps>(
            this, "/jixiebi/learning/count_steps",
            // ① 审查新目标。已有任务时拒绝新的请求，避免覆盖正在执行的目标句柄。
            [this](const rclcpp_action::GoalUUID &,
                   std::shared_ptr<const CountSteps::Goal> goal) {
                if (active_goal_ || goal->total_steps < 1 || goal->total_steps > 20) {
                    RCLCPP_WARN(get_logger(), "拒绝目标：请求%u步，当前%s，允许1～20步",
                                static_cast<unsigned>(goal->total_steps),
                                active_goal_ ? "忙" : "空闲");
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            // ② 同意取消正在执行的这个目标。ROS随后将其转为CANCELING。
            [this](const std::shared_ptr<ServerGoalHandle> goal_handle) {
                if (goal_handle != active_goal_) {
                    return rclcpp_action::CancelResponse::REJECT;
                }
                RCLCPP_INFO(get_logger(), "接受取消请求，已完成%u步",
                            static_cast<unsigned>(completed_steps_));
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            // ③ ROS已接受目标，用句柄给这个客户端回传进度和最终结果。
            [this](const std::shared_ptr<ServerGoalHandle> goal_handle) {
                active_goal_ = goal_handle;
                completed_steps_ = 0;
                timer_->reset();
                RCLCPP_INFO(get_logger(), "开始模拟任务，目标%u步",
                            static_cast<unsigned>(goal_handle->get_goal()->total_steps));
            });
        RCLCPP_INFO(get_logger(), "模拟计数Action就绪：/jixiebi/learning/count_steps");
    }

private:
    void advance_one_step() {
        // 取消已获接受时，报告截至上一拍的步数，然后释放任务位置。
        if (active_goal_->is_canceling()) {
            auto result = std::make_shared<CountSteps::Result>();
            result->completed_steps = completed_steps_;
            active_goal_->canceled(result);
            RCLCPP_INFO(get_logger(), "CANCELED：完成%u步", static_cast<unsigned>(completed_steps_));
        } else {
            ++completed_steps_;
            const auto total = active_goal_->get_goal()->total_steps;
            auto feedback = std::make_shared<CountSteps::Feedback>();
            feedback->completed_steps = completed_steps_;
            active_goal_->publish_feedback(feedback);  // 向这个目标的客户端报告模拟进度。
            RCLCPP_INFO(get_logger(), "进度 %u/%u", static_cast<unsigned>(completed_steps_),
                        static_cast<unsigned>(total));
            if (completed_steps_ < total) {
                return;  // 保持定时器运行，下一拍继续。
            }
            auto result = std::make_shared<CountSteps::Result>();
            result->completed_steps = completed_steps_;
            active_goal_->succeed(result);
            RCLCPP_INFO(get_logger(), "SUCCEEDED：完成%u步", static_cast<unsigned>(completed_steps_));
        }
        timer_->cancel();
        active_goal_.reset();  // 终态结果已交给ROS，下一次请求可以开始新的计数。
    }

    rclcpp_action::Server<CountSteps>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<ServerGoalHandle> active_goal_;
    std::uint32_t completed_steps_{0};  // 从当前目标开始累计的模拟步数。
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        // 单线程执行器顺序调度回调，同一时刻只有一个回调访问active_goal_。
        rclcpp::spin(std::make_shared<CountStepsServer>());
    } catch (const std::exception &error) {
        std::cerr << "SERVER_ERROR: " << error.what() << '\n';
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
