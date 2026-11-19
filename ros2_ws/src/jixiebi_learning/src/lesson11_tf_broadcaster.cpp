#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using TransformStamped = geometry_msgs::msg::TransformStamped;
using namespace std::chrono_literals;

class LessonTfBroadcaster : public rclcpp::Node {
public:
    LessonTfBroadcaster()
        : Node("jixiebi_lesson11_tf"), static_broadcaster_(*this),
          dynamic_broadcaster_(*this), start_time_(now()) {
        publish_base();
        // 定时器每 50 ms 触发一次，由 spin 调度回调，约为 20 Hz。
        timer_ = create_wall_timer(50ms, [this]() { publish_tool(); });
        RCLCPP_INFO(get_logger(), "坐标树已启动：lesson11_world → lesson11_base → lesson11_tool");
    }

private:
    void publish_base() {
        TransformStamped base;
        // 父系是观察的基准；translation 表示子系原点在父系中的坐标（米）。
        base.header.frame_id = "lesson11_world";
        base.child_frame_id = "lesson11_base";
        base.header.stamp = now();
        base.transform.translation.x = 0.10;
        base.transform.translation.z = 0.05;
        base.transform.rotation.w = 1.0;  // (x,y,z,w)=(0,0,0,1)，两组轴朝向一致。
        // 基座的位置固定。广播器作为成员持续存在，晚加入的监听器也能读到 /tf_static。
        static_broadcaster_.sendTransform(base);
    }

    void publish_tool() {
        // 用同一个 ROS 时刻算姿态、填时间戳；定时器偶尔晚到也按实际经过的秒数计算。
        const auto stamp = now();
        constexpr double radius_m = 0.10;
        constexpr double angular_speed_rad_s = 0.5;
        const double theta = angular_speed_rad_s * (stamp - start_time_).seconds();

        TransformStamped tool;
        // 这条边以 base 为父系。沿 base 的 XY 平面绕原点画圆，局部 z 始终为 0。
        tool.header.frame_id = "lesson11_base";
        tool.child_frame_id = "lesson11_tool";
        tool.header.stamp = stamp;
        tool.transform.translation.x = radius_m * std::cos(theta);
        tool.transform.translation.y = radius_m * std::sin(theta);
        // 工具同时绕 +Z 转过 theta，局部 +X 轴指向圆外。纯 Z 轴旋转的四元数用半角。
        tool.transform.rotation.z = std::sin(theta / 2.0);
        tool.transform.rotation.w = std::cos(theta / 2.0);
        dynamic_broadcaster_.sendTransform(tool);  // 每帧经 /tf 发布给监听器。
    }

    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    tf2_ros::TransformBroadcaster dynamic_broadcaster_;
    rclcpp::Time start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LessonTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
