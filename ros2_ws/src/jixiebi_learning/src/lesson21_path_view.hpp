#pragma once

#include "kdl/frames.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>

namespace lesson_line {
using Marker = visualization_msgs::msg::Marker;
using Markers = visualization_msgs::msg::MarkerArray;

inline geometry_msgs::msg::Point point(const KDL::Vector &v)
{
    geometry_msgs::msg::Point p;
    p.x = v.x(); p.y = v.y(); p.z = v.z();
    return p;
}

class PathView {
public:
    PathView(const rclcpp::Node::SharedPtr &node, KDL::Vector start, KDL::Vector finish)
        : start_(start), finish_(finish)
    {
        pub_ = node->create_publisher<Markers>("/jixiebi/learning/line_trace", rclcpp::QoS(1).transient_local());
        markers_.markers.resize(2);
        auto &reference = markers_.markers[0];
        reference.header.frame_id = "base_link";
        reference.ns = "lesson21";
        reference.pose.orientation.w = 1.0;
        reference.type = Marker::SPHERE_LIST;
        reference.scale.x = reference.scale.y = reference.scale.z = 0.006;
        reference.color.b = reference.color.a = 1.0;
        reference.color.g = 0.65;
        // 蓝点直接取自几何直线，每隔 1/20 线长画一点，和求逆解时分几段无关。
        for (int i = 0; i <= 20; ++i) {
            reference.points.push_back(point(start_ + (finish_ - start_) * (i / 20.0)));
        }
        auto &trace = markers_.markers[1];
        trace = reference;
        trace.points.clear();
        trace.id = 1;
        trace.type = Marker::LINE_STRIP;
        trace.scale.x = 0.0025;
        trace.color.r = 1.0; trace.color.g = 0.32; trace.color.b = 0.04;
        trace.action = Marker::DELETE;  // 再次练习时清掉上一次的橙线。
        pub_->publish(markers_);
    }

    void observe(const KDL::Vector &tip)
    {
        auto &trace = markers_.markers[1];
        trace.points.push_back(point(tip));
        // 到有限线段的最短距离。偏差只衡量轨迹形状，与此帧走得快慢无关。
        const auto line = finish_ - start_;
        const double u = std::clamp(KDL::dot(tip - start_, line) / KDL::dot(line, line), 0.0, 1.0);
        const double mm = (tip - (start_ + line * u)).Norm() * 1000.0;
        max_error_mm = std::max(max_error_mm, mm);
        if (trace.points.size() >= 2) {
            trace.action = Marker::ADD;
            pub_->publish(markers_);
        }
    }

    size_t samples() const { return markers_.markers[1].points.size(); }
    double max_error_mm = 0.0;

private:
    KDL::Vector start_, finish_;
    Markers markers_;
    rclcpp::Publisher<Markers>::SharedPtr pub_;
};
}  // namespace lesson_line
