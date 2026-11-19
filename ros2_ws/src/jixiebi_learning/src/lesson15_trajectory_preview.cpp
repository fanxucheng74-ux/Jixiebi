#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "trajectory_playback.hpp"

#include <cmath>
#include <cstdlib>
#include <memory>

using lesson_trajectory::make_preview;
using lesson_trajectory::Trajectory;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lesson15_trajectory_preview");

    // 轨迹只在启动时构造一次。范围取自第十二课模型中肘关节的 ±90°。
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    const double offset = node->declare_parameter<double>("elbow_offset_deg", -20.0, startup_only);
    if (!std::isfinite(offset) || std::abs(offset) > 90.0) {
        RCLCPP_ERROR(node->get_logger(), "elbow_offset_deg 须为 [-90, 90] 内的有限角度（度）");
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // 第五步：只向课程预览话题发一次。发布器存活期间，稍后打开的终端也能取到这一条。
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto publisher = node->create_publisher<Trajectory>("/jixiebi/learning/trajectory_preview", qos);
    publisher->publish(make_preview(offset));
    RCLCPP_INFO(node->get_logger(), "轨迹预览已发布：3 个点，0 / 2 / 4 秒，肘关节偏转 %.2f°", offset);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
