#include "arm_state.hpp"

#include "jixiebi_interfaces/msg/servo_states_raw.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using RawState = jixiebi_interfaces::msg::ServoStateRaw;
using RawBatch = jixiebi_interfaces::msg::ServoStatesRaw;
static_assert(jixiebi::kServoCount == 6U, "原始ROS接口固定六项，改变数量时必须同步接口");

// 显式映射两个接口的状态名，不依赖native枚举与ROS常量碰巧具有相同数字。
std::uint8_t message_status(jixiebi::SampleStatus status)
{
    switch (status) {
    case jixiebi::SampleStatus::Ok:
        return RawState::OK;
    case jixiebi::SampleStatus::Failed:
        return RawState::FAILED;
    default:
        return RawState::NOT_QUERIED;
    }
}

// 节点类继承rclcpp::Node，保留ROS能力；硬件查询仍交给已有native核心。
class ServoStatePublisher : public rclcpp::Node {
public:
    ServoStatePublisher() : Node("jixiebi_servo_state_cpp")
    {
        // 这些参数仅在启动时读取。标成只读，避免param set成功但驱动实际未重配置。
        rcl_interfaces::msg::ParameterDescriptor startup_only;
        startup_only.read_only = true;
        const auto port = declare_parameter<std::string>("port", "", startup_only);
        const auto period_ms = declare_parameter<std::int64_t>("period_ms", 1000, startup_only);
        query_timeout_ms_ =
            declare_parameter<std::int64_t>("query_timeout_ms", 500, startup_only);
        const bool pty_simulation = declare_parameter<bool>("pty_simulation", false, startup_only);
        if (port.empty()) {
            throw std::invalid_argument("必须显式指定-p port:=设备路径；默认不打开串口");
        }
        if (period_ms < 100 || period_ms > 60000 ||
            query_timeout_ms_ < 50 || query_timeout_ms_ > 5000) {
            throw std::invalid_argument("period_ms须为100～60000，query_timeout_ms须为50～5000");
        }
        source_ = pty_simulation ? "PTY_SIMULATION" : "SERIAL_TTY";

        // 一个节点只拥有一个串口对象；unique_ptr负责析构释放fd和协作锁。
        reader_ = std::make_unique<jixiebi::PositionReader>(port, pty_simulation);
        // 可靠、只保留最新一条、volatile不向晚加入者回放旧状态。
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        publisher_ = create_publisher<RawBatch>("/jixiebi/servo_states_cpp", qos);
        // 这里只登记回调，spin开始调度后才执行。每批完成后重新计间隔，避免追赶积压。
        timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() {
            publish_state();
        });
        RCLCPP_INFO(get_logger(), "只读节点就绪：%s，完成每批后间隔%lldms，话题%s",
                    port.c_str(), static_cast<long long>(period_ms), publisher_->get_topic_name());
    }

private:
    void publish_state()
    {
        timer_->cancel();  // 本批期间停表；失败后保持暂停，不自动补查或重连。
        if (!rclcpp::ok()) {
            return;
        }
        try {
            const auto batch = jixiebi::read_arm_state(
                *reader_, std::chrono::milliseconds(query_timeout_ms_));
            if (!rclcpp::ok()) {
                return;  // Ctrl+C后不再发布/重新启动定时器。
            }
            RawBatch message{};  // 每批新建，不能把上一批成功P遗留到这批失败位置。
            message.stamp = now();  // ROS节点发布前的时间，不是舵机采样时间。
            message.source = source_;
            message.sequence = ++sequence_;
            message.simultaneous = false;
            message.complete = batch.complete;
            message.valid_count = static_cast<std::uint8_t>(batch.valid_count);
            message.elapsed_ms = batch.elapsed_ms;
            for (std::size_t index = 0U; index < batch.samples.size(); ++index) {
                const auto &sample = batch.samples[index];
                auto &output = message.servos[index];
                output.servo_id = sample.servo_id;
                output.status = message_status(sample.status);
                if (sample.status == jixiebi::SampleStatus::Ok) {
                    output.position_p = sample.position_p;  // 协议P，不写进弧度JointState。
                }
                output.elapsed_ms = sample.elapsed_ms;
                output.finished_offset_ms = sample.finished_offset_ms;
                output.error = sample.error;
            }
            publisher_->publish(message);  // 将类型化消息交给ROS，不再仅打印到终端。
            RCLCPP_INFO(get_logger(), "发布批次%llu，有效%u/6，耗时%.3fms，complete=%s",
                        static_cast<unsigned long long>(message.sequence),
                        static_cast<unsigned int>(message.valid_count), message.elapsed_ms,
                        message.complete ? "true" : "false");
            if (batch.complete) {
                timer_->reset();  // 下一批从现在起再等period_ms，不是严格固定频率。
            } else {
                RCLCPP_ERROR(get_logger(), "本批不完整，查询已暂停；请Ctrl+C退出并核对错误后手动启动");
            }
        } catch (const std::exception &error) {
            // 发布等异常也不能触发串口自动重查；节点仍在，保留日志/锁直到用户退出。
            RCLCPP_ERROR(get_logger(), "查询/发布异常，已暂停：%s", error.what());
        }
    }

    std::unique_ptr<jixiebi::PositionReader> reader_;
    rclcpp::Publisher<RawBatch>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string source_;
    std::int64_t query_timeout_ms_ = 500;
    std::uint64_t sequence_ = 0U;  // 本节点批次序号，重启归零，不是舵机请求序号。
};

}  // namespace

int main(int argc, char *argv[])
{
    try {
        rclcpp::init(argc, argv);  // 初始化ROS并解析--ros-args参数。
        // spin在当前线程调度定时回调，串口查询不并发；Ctrl+C后退出并析构节点。
        rclcpp::spin(std::make_shared<ServoStatePublisher>());
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
