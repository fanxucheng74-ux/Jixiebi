#include "lesson_timed_line.hpp"
#include "lesson22_speed_profile.hpp"

namespace {
using namespace lesson_line;
using lesson_speed::Profile;

void run(const rclcpp::Node::SharedPtr &node)
{
    const double v = node->declare_parameter<double>("cruise_mm_s", 35.0);
    // 本例在这段速度范围内始终有巡航段；数值仅供这条 WSL 模型路径练习。
    if (!std::isfinite(v) || v < 10.0 || v > 50.0) throw std::invalid_argument("cruise_mm_s 须为 10～50 mm/s");
    const Profile profile{(right - left).Norm() * 1000.0, v};
    lesson_speed::SpeedMeter meter(profile);
    std::cout << "TARGET_MM_S=" << v << " PLANNED_LINE_S=" << profile.duration() << std::endl;
    run_model_line(node, "/jixiebi/learning/sampled_follow_joint_trajectory",
        [&](const KDL::Chain &chain, const KDL::JntArray &current, double &entry_s) {
            return make_timed_line(chain, current, profile, entry_s);
        },
        [&](double t, const KDL::Vector &tip) { meter.sample(t, tip); });
    meter.report();
}
}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        run(std::make_shared<rclcpp::Node>("lesson22_timed_line"));
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
