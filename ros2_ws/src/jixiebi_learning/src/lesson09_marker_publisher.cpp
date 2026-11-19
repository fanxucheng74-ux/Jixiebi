#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using Marker = visualization_msgs::msg::Marker;
using namespace std::chrono_literals;

class BlockMarkerPublisher : public rclcpp::Node {
public:
    BlockMarkerPublisher() : Node("jixiebi_lesson09_marker") {
        // 发布者保留最近一个样本，使用 transient_local 的订阅者可在加入时接收它。
        // 每秒也会发布新样本，因此普通 volatile 订阅者加入后也能收到下一条。
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        publisher_ = create_publisher<Marker>("/jixiebi/learning/block_marker", qos);
        timer_ = create_wall_timer(1s, [this]() { publish_block(); });
        RCLCPP_INFO(get_logger(), "方块发布器就绪，话题 %s，RViz 固定坐标系 lesson09_world",
                    publisher_->get_topic_name());
    }

private:
    void publish_block() {
        Marker marker;
        // RViz 按 frame_id 解释下方坐标，stamp 使用本次发布时的 ROS 时间。
        marker.header.frame_id = "lesson09_world";
        marker.header.stamp = now();
        marker.ns = "blocks";
        marker.id = 0;  // ns 与 id 共同标识图形，多次 ADD 会更新同一个方块。
        marker.type = Marker::CUBE;
        marker.action = Marker::ADD;

        // 尺寸和位置都以米为单位。pose 指定中心，把中心抬高半个边长，底面恰在 z=0。
        constexpr double side_m = 0.025;  // 25 mm
        marker.scale.x = side_m;
        marker.scale.y = side_m;
        marker.scale.z = side_m;
        marker.pose.position.x = 0.20;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = side_m / 2.0;
        marker.pose.orientation.w = 1.0;  // 配合默认 x=y=z=0，单位四元数表示零旋转。

        // RGBA 各分量范围为 0～1。这里红色取满，alpha=1 让方块完全不透明。
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        publisher_->publish(marker);
    }

    rclcpp::Publisher<Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // spin 调度上面登记的定时回调，Ctrl+C 结束节点。
    rclcpp::spin(std::make_shared<BlockMarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}
