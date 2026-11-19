#include "calibrated_feedback.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

class JointStateBridge : public rclcpp::Node {
public:
    JointStateBridge() : Node("joint_state_bridge")
    {
        // 四个角坐标沿用第十二课 URDF 的关节零位和正轴方向，配置单位是度。
        rcl_interfaces::msg::ParameterDescriptor startup_only;
        startup_only.read_only = true;
        const double missing_angle = std::numeric_limits<double>::quiet_NaN();
        if (declare_parameter<bool>("use_nominal_model", false, startup_only)) {
            const auto p = declare_parameter<std::int64_t>("yaw_zero_p", 1500, startup_only);
            if (p < 500 || p > 2500) {
                throw std::invalid_argument("底座工作零位须是协议范围 500～2500 内的 P 值");
            }
            nominal_yaw_zero_p_ = static_cast<int>(p);
        } else {
            read_measured_parameters(startup_only, missing_angle);
        }
        // 两种配置不能混用，也不能静默忽略拼错的键。
        for (const auto &entry : get_node_parameters_interface()->get_parameter_overrides()) {
            if (!has_parameter(entry.first)) {
                throw std::invalid_argument("未识别的启动参数：" + entry.first);
            }
        }
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        const char *topic = nominal_yaw_zero_p_ ? "/nominal_estimate/joint_states" : "/joint_states";
        publisher_ = create_publisher<jixiebi::JointState>(topic, qos);
        subscription_ = create_subscription<jixiebi::RawBatch>("/jixiebi/servo_states_cpp", qos,
            [this](jixiebi::RawBatch::ConstSharedPtr raw) { publish_feedback(*raw); });
        RCLCPP_INFO(get_logger(), "等待 RK 六轴反馈，%s → %s",
                    nominal_yaw_zero_p_ ? "厂家标称角估计" : "本机两点标定", topic);
    }

private:
    void read_measured_parameters(const rcl_interfaces::msg::ParameterDescriptor &startup_only,
                                  double missing_angle)
    {
        for (std::size_t id = 0; id < calibrations_.size(); ++id) {
            const std::string key = std::string(jixiebi::kJointNames[id]) + ".";
            const auto servo_id = declare_parameter<std::int64_t>(key + "servo_id", -1, startup_only);
            const auto p_a = declare_parameter<std::int64_t>(key + "p_a", -1, startup_only);
            const auto angle_a = declare_parameter<double>(key + "angle_a_deg", missing_angle, startup_only);
            const auto p_b = declare_parameter<std::int64_t>(key + "p_b", -1, startup_only);
            const auto angle_b = declare_parameter<double>(key + "angle_b_deg", missing_angle, startup_only);
            calibrations_[id] = jixiebi::checked_calibration(id, servo_id, p_a, angle_a, p_b, angle_b);
        }
        const auto count = std::count_if(calibrations_.begin(), calibrations_.end(),
                                        [](const auto &item) { return item.has_value(); });
        if (count == 0) {
            throw std::invalid_argument("至少需要一个关节的实测两点标定；请填写自己的 ROS 参数文件");
        }
    }

    void publish_feedback(const jixiebi::RawBatch &raw)
    {
        jixiebi::JointState output;
        if (const char *error = jixiebi::convert_feedback(raw, calibrations_, output, nominal_yaw_zero_p_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "本批无输出：%s", error);
            return;
        }
        // 上游重启后 sequence 可从 1 重新计数。只比同一来源的批次时间，不依赖两机校时。
        const auto &stamp = output.header.stamp;
        if (stamp.sec < last_stamp_.sec ||
            (stamp.sec == last_stamp_.sec && stamp.nanosec <= last_stamp_.nanosec)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "本批 RK 时间戳重复或倒序，跳过");
            return;
        }
        publisher_->publish(output);
        last_stamp_ = stamp;
    }

    jixiebi::Calibrations calibrations_{};
    std::optional<int> nominal_yaw_zero_p_;
    builtin_interfaces::msg::Time last_stamp_{};
    rclcpp::Publisher<jixiebi::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<jixiebi::RawBatch>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char *argv[])
{
    try {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<JointStateBridge>());
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
}
