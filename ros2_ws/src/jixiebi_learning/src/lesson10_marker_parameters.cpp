#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using Marker = visualization_msgs::msg::Marker;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;
using namespace std::chrono_literals;

class ParameterMarkerPublisher : public rclcpp::Node {
public:
    ParameterMarkerPublisher() : Node("jixiebi_lesson10_marker") {
        // 先登记校验回调，再声明参数。Humble 在声明时也检查 Launch 提供的初值。
        // 把句柄存为成员，让运行期间的每次改参都经过同一个回调。
        validation_handle_ = add_on_set_parameters_callback(validate_parameters);
        declare_parameter<std::string>("color", "green");
        declare_parameter<double>("side_m", 0.025);  // 单位为米，默认边长 25 mm。

        // 沿用第九课的话题及 QoS，RViz2 可以用原来的视图显示同一个方块。
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        publisher_ = create_publisher<Marker>("/jixiebi/learning/block_marker", qos);
        timer_ = create_wall_timer(1s, [this]() { publish_block(); });
        RCLCPP_INFO(get_logger(), "方块发布器就绪，可在线修改 color 和 side_m");
    }

private:
    static SetParametersResult validate_parameters(const std::vector<rclcpp::Parameter> &changes) {
        // ROS 已核对声明的类型。这里只检查候选值；原子请求里有一项不合格就拒绝整批。
        // 提交由 ROS 完成，回调中直接改颜色或发布消息会让被拒绝的值提前显示。
        SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : changes) {
            if (parameter.get_name() == "color") {
                const auto &color = parameter.as_string();
                if (color != "red" && color != "green" && color != "blue") {
                    result.successful = false;
                    result.reason = "color 只接受 red、green、blue";
                    return result;
                }
            } else if (parameter.get_name() == "side_m") {
                const double side_m = parameter.as_double();
                if (!std::isfinite(side_m) || side_m < 0.01 || side_m > 0.10) {
                    result.successful = false;
                    result.reason = "side_m 需为 0.01 至 0.10 米之间的有限数，含两端";
                    return result;
                }
            }
        }
        return result;
    }

    void publish_block() {
        // spin 使用单线程执行器。每一拍读取 ROS 已提交的参数，再生成这一帧。
        const auto color = get_parameter("color").as_string();
        const double side_m = get_parameter("side_m").as_double();
        Marker marker;
        marker.header.frame_id = "lesson09_world";
        marker.header.stamp = now();
        marker.ns = "blocks";
        marker.id = 0;  // 使用相同的 ns 和 id，让新消息更新原来的方块。
        marker.type = Marker::CUBE;
        marker.action = Marker::ADD;

        marker.scale.x = side_m;
        marker.scale.y = side_m;
        marker.scale.z = side_m;
        marker.pose.position.x = 0.20;
        marker.pose.position.z = side_m / 2.0;  // 中心随边长升降，底面始终贴住网格。
        marker.pose.orientation.w = 1.0;        // 其余分量默认是零，方块与坐标轴对齐。
        // 选中的颜色通道取 1，其余取 0；alpha 保持完全不透明。
        marker.color.r = color == "red" ? 1.0f : 0.0f;
        marker.color.g = color == "green" ? 1.0f : 0.0f;
        marker.color.b = color == "blue" ? 1.0f : 0.0f;
        marker.color.a = 1.0f;
        publisher_->publish(marker);
    }

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validation_handle_;
    rclcpp::Publisher<Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        // 定时器与参数服务都靠 spin 调度；改参命令可在节点运行时得到响应。
        rclcpp::spin(std::make_shared<ParameterMarkerPublisher>());
    } catch (const std::exception &error) {
        // 例如启动时给出无效边长，报告原因并让启动方看到失败退出码。
        std::cerr << "MARKER_ERROR: " << error.what() << '\n';
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
