#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_playback.hpp"

#include <chrono>
#include <memory>
#include <vector>

using lesson_trajectory::check_preview;
using lesson_trajectory::positions_at;
using lesson_trajectory::seconds_at;
using lesson_trajectory::Trajectory;
using JointState = sensor_msgs::msg::JointState;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class TrajectoryPlayer : public rclcpp::Node {
public:
    TrajectoryPlayer() : Node("lesson16_trajectory_player")
    {
        states_ = create_publisher<JointState>("/jixiebi/learning/playback/joint_states", 10);
        // 第十五课只发一次。匹配发布端的 QoS，发布端仍运行时，晚启动也能收到。
        preview_ = create_subscription<Trajectory>(
            "/jixiebi/learning/trajectory_preview", rclcpp::QoS(1).reliable().transient_local(),
            [this](Trajectory::ConstSharedPtr msg) { receive(msg); });
        timer_ = create_wall_timer(20ms, [this]() { tick(); });
        RCLCPP_INFO(get_logger(), "等待第十五课轨迹");
    }

private:
    void receive(Trajectory::ConstSharedPtr msg)
    {
        // 播放中收到另一条消息时保留当前动作，避免模型突然跳回新起点。
        if (playing_) {
            RCLCPP_WARN(get_logger(), "当前轨迹正在播放，忽略本次预览");
            return;
        }
        const auto error = check_preview(*msg);
        if (!error.empty()) {
            RCLCPP_WARN(get_logger(), "忽略预览：%s", error.c_str());
            return;
        }
        path_ = msg;
        start_ = Clock::now();
        playing_ = true;
        publish_positions(path_->points.front().positions);
        RCLCPP_INFO(get_logger(), "开始播放 %zu 个路点，全长 %.3f 秒", path_->points.size(),
                    seconds_at(path_->points.back()));
    }

    void publish_positions(const std::vector<double> &positions)
    {
        JointState state;
        state.header.stamp = now();  // ROS 时间戳供状态发布器生成 TF。
        state.name = path_->joint_names;
        state.position = positions;  // 同下标配对，角度单位为 rad。
        states_->publish(state);
    }

    void tick()
    {
        if (!path_) return;  // 收到首条轨迹前没有可以显示的姿态。
        // 单调时钟决定播到哪里。某一帧晚到，下一帧仍跟着实际经过时间走。
        const double elapsed = std::chrono::duration<double>(Clock::now() - start_).count();
        if (elapsed >= seconds_at(path_->points.back())) {
            publish_positions(path_->points.back().positions);
            if (playing_) RCLCPP_INFO(get_logger(), "播放完成，保持最后姿态");
            playing_ = false;
            return;
        }
        publish_positions(positions_at(*path_, elapsed));
    }

    Trajectory::ConstSharedPtr path_;
    Clock::time_point start_;
    bool playing_ = false;
    rclcpp::Publisher<JointState>::SharedPtr states_;
    rclcpp::Subscription<Trajectory>::SharedPtr preview_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 单线程 spin 依次调度订阅和定时器回调，两者共享的轨迹不会被同时修改。
    rclcpp::spin(std::make_shared<TrajectoryPlayer>());
    rclcpp::shutdown();
    return 0;
}
