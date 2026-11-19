#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_playback.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <memory>
#include <string>

using Follow = control_msgs::action::FollowJointTrajectory;
using Handle = rclcpp_action::ServerGoalHandle<Follow>;
using State = sensor_msgs::msg::JointState;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class FollowPlayer : public rclcpp::Node {
public:
    FollowPlayer() : Node("lesson17_follow_player")
    {
        // 模式由 launch 在启动时选定，运行中保持同一套接单规则。
        rcl_interfaces::msg::ParameterDescriptor startup_only;
        startup_only.read_only = true;
        check_start_ = declare_parameter<bool>("check_start", false, startup_only);
        sampled_linear_ = declare_parameter<bool>("sampled_linear", false, startup_only);
        // 初次打开 RViz 时就显示课程的参考姿态。此状态只存在于电脑模型中。
        const auto example = lesson_trajectory::make_preview(0.0);
        frame_.name = example.joint_names;
        frame_.position = example.points.front().positions;
        states_ = create_publisher<State>("/jixiebi/learning/follow/joint_states", 10);
        server_ = rclcpp_action::create_server<Follow>(
            this, sampled_linear_ ? "/jixiebi/learning/sampled_follow_joint_trajectory"
                                  : "/jixiebi/learning/follow_joint_trajectory",
            [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> goal) {
                return review(*goal);
            },
            [this](std::shared_ptr<Handle> handle) {
                // 只受理属于当前目标的取消，终态在下一次 tick 中提交。
                return handle == active_ ? rclcpp_action::CancelResponse::ACCEPT
                                         : rclcpp_action::CancelResponse::REJECT;
            },
            [this](std::shared_ptr<Handle> handle) {
                active_ = handle;
                start_ = Clock::now();
                render(0.0);  // 新目标的第一个路点明确指定这一遍的起始姿态。
                RCLCPP_INFO(get_logger(), "开始播放模型轨迹");
            });
        timer_ = create_wall_timer(20ms, [this]() { tick(); });
        RCLCPP_INFO(get_logger(), "模型 Action 就绪，等待轨迹目标；起点核对%s", check_start_ ? "已开启" : "已关闭");
    }

private:
    rclcpp_action::GoalResponse review(const Follow::Goal &goal)
    {
        // 本课实现位置轨迹子集，不能默默忽略客户端要求的多自由度轨迹或跟踪容差。
        std::string reason = lesson_trajectory::check_preview(goal.trajectory);
        if (reason.empty() && sampled_linear_) {
            // 线性段的关节速度就是两点差除以秒数。每段均须满足 URDF 的 1 rad/s。
            const auto &points = goal.trajectory.points;
            for (size_t i = 1; i < points.size(); ++i) {
                const double dt = lesson_trajectory::seconds_at(points[i]) - lesson_trajectory::seconds_at(points[i-1]);
                for (size_t j = 0; j < points[i].positions.size(); ++j) {
                    if (std::abs(points[i].positions[j] - points[i-1].positions[j]) > dt + 1e-9)
                        reason = "采样轨迹超过模型关节速度上限";
                }
            }
        }
        if (reason.empty() && check_start_) reason = start_mismatch(goal.trajectory);
        if (active_) reason = "已有目标在播放";
        const auto &multi = goal.multi_dof_trajectory;
        if (!multi.points.empty() || !multi.joint_names.empty()) reason = "模型只支持单自由度关节";
        if (!goal.path_tolerance.empty() || !goal.goal_tolerance.empty() ||
            !goal.component_path_tolerance.empty() || !goal.component_goal_tolerance.empty() ||
            goal.goal_time_tolerance.sec != 0 || goal.goal_time_tolerance.nanosec != 0) {
            reason = "本课只支持默认容差";
        }
        if (!reason.empty()) {
            RCLCPP_WARN(get_logger(), "拒绝目标：%s", reason.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    std::string start_mismatch(const lesson_trajectory::Trajectory &path) const
    {
        // 采样后姿态可能已变。按名字核对全部四轴，允许消息的列顺序不同。
        // 1e-6 rad 只用于同一理想模型的数值比较，实机精度需另测。
        for (std::size_t i = 0; i < path.joint_names.size(); ++i) {
            const auto &name = path.joint_names[i];
            const auto it = std::find(frame_.name.begin(), frame_.name.end(), name);
            if (it == frame_.name.end()) return "当前模型缺少关节 " + name;
            const double current = frame_.position[std::distance(frame_.name.begin(), it)];
            if (std::abs(path.points.front().positions[i] - current) > 1e-6) {
                return "轨迹起点与当前模型不符：" + name;
            }
        }
        return {};
    }

    void publish_frame()
    {
        frame_.header.stamp = now();
        states_->publish(frame_);
    }

    void render(double seconds)
    {
        const auto &path = active_->get_goal()->trajectory;
        frame_.name = path.joint_names;
        frame_.position = lesson_trajectory::positions_at(path, seconds, sampled_linear_);
        publish_frame();

        // desired 是插值期望；actual 是刚交给模型显示的角度，两者都使用 rad。
        // 这个理想播放器直接应用每一帧，因此误差为零，没有采集编码器反馈。
        auto feedback = std::make_shared<Follow::Feedback>();
        feedback->header = frame_.header;
        feedback->joint_names = frame_.name;
        feedback->desired.time_from_start = rclcpp::Duration::from_seconds(seconds);
        feedback->desired.positions = frame_.position;
        feedback->actual = feedback->desired;
        feedback->error.time_from_start = feedback->desired.time_from_start;
        feedback->error.positions.assign(frame_.name.size(), 0.0);
        active_->publish_feedback(feedback);
    }

    void tick()
    {
        if (!active_) {
            publish_frame();  // 等待、完成和取消后，继续给 RViz 提供当前姿态与新时间戳。
            return;
        }
        if (active_->is_canceling()) {
            // 先检查取消再插值：保留最后已经发布的角度，随后不再沿旧轨迹推进。
            auto result = std::make_shared<Follow::Result>();
            result->error_string = "模型保持最后发布的姿态";
            active_->canceled(result);
            active_.reset();
            RCLCPP_INFO(get_logger(), "CANCELED，模型已保持");
            return;
        }
        const double end = lesson_trajectory::seconds_at(active_->get_goal()->trajectory.points.back());
        const double elapsed = std::chrono::duration<double>(Clock::now() - start_).count();
        render(std::min(elapsed, end));
        if (elapsed >= end) {
            auto result = std::make_shared<Follow::Result>();
            result->error_code = Follow::Result::SUCCESSFUL;
            active_->succeed(result);
            active_.reset();
            RCLCPP_INFO(get_logger(), "SUCCEEDED，模型已到末点");
        }
    }

    bool check_start_;
    bool sampled_linear_;
    std::shared_ptr<Handle> active_;
    Clock::time_point start_;
    State frame_;
    rclcpp::Publisher<State>::SharedPtr states_;
    rclcpp_action::Server<Follow>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 用单线程执行器顺序处理目标、取消与定时器，不用睡眠或后台运动线程。
    rclcpp::spin(std::make_shared<FollowPlayer>());
    rclcpp::shutdown();
    return 0;
}
