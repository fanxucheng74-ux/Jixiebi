#include "lesson23_ramp_profile.hpp"
#include "lesson_timed_line.hpp"

#include <filesystem>
#include <fstream>

namespace {
using lesson_ramp::RampProfile;
using lesson_ramp::Shape;
namespace fs = std::filesystem;

struct TipSample {
    double t;     // Action 横移反馈时刻，s；左端开始计时。
    double x_mm;  // 球心在 base_link 中的 X，沿本条直线向右为正。
};

std::ofstream open_csv(const fs::path &path)
{
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(path);
    out << std::setprecision(12);
    return out;
}

// 理论曲线直接对时间律求值，与模型反馈的时间间隔独立。
void write_reference(const fs::path &file, const RampProfile &p)
{
    auto out = open_csv(file);
    out << "t_s,v_mm_s,a_mm_s2\n";
    auto row = [&](double t) {
        out << t << ',' << p.speed_mm_s(t) << ',' << p.accel_mm_s2(t) << '\n';
    };
    row(0.0);
    const double brake = p.duration() - p.ramp_s;
    lesson_line::sample_interval(0.0, p.ramp_s, 0.005, row);
    lesson_line::sample_interval(p.ramp_s, brake, 0.005, row);
    lesson_line::sample_interval(brake, p.duration(), 0.005, row);
    out.close();
}

// 相邻位置给出区间平均速度，速度放到区间中点；两次速度差再除以中点的真实时间差。
void write_feedback(const fs::path &file, const std::vector<TipSample> &samples)
{
    if (samples.size() < 3) throw std::runtime_error("至少需要三帧球心反馈才能估算加速度");
    auto out = open_csv(file);
    out << "t_x_s,x_mm,t_v_s,v_mm_s,t_a_s,a_mm_s2\n";
    double prior_v = 0.0, prior_tv = 0.0;
    double peak = std::numeric_limits<double>::lowest(), trough = std::numeric_limits<double>::max();
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto &now = samples[i];
        out << now.t << ',' << now.x_mm;
        if (i == 0) { out << ",,,,\n"; continue; }  // 首帧尚没有可相减的位置。
        const auto &before = samples[i - 1];
        const double dt = now.t - before.t;
        if (dt <= 0.0) throw std::runtime_error("反馈时间须严格递增");
        const double v = (now.x_mm - before.x_mm) / dt;
        const double tv = (now.t + before.t) / 2.0;
        out << ',' << tv << ',' << v;
        if (i >= 2) {
            const double a = (v - prior_v) / (tv - prior_tv);
            out << ',' << (tv + prior_tv) / 2.0 << ',' << a;
            peak = std::max(peak, a);
            trough = std::min(trough, a);
        } else {
            out << ",,";  // 第二帧首次得到速度；加速度仍然留空。
        }
        out << '\n';
        prior_v = v;
        prior_tv = tv;
    }
    out.close();
    std::cout << "MODEL_MAX_AX_MM_S2=" << peak << " MODEL_MIN_AX_MM_S2=" << trough << '\n';
}

void compare(const rclcpp::Node::SharedPtr &node)
{
    const auto v = node->declare_parameter<double>("cruise_mm_s", 28.0);
    if (!std::isfinite(v) || v < 10.0 || v > 50.0) throw std::invalid_argument("巡航速度范围为 10～50 mm/s");
    const fs::path folder = node->declare_parameter<std::string>("output_dir", "lesson23_curves");
    if (!fs::create_directory(folder)) throw std::runtime_error("请为 output_dir 指定尚未使用的新目录");

    for (auto shape : {Shape::Trapezoid, Shape::Smooth}) {
        const std::string name = shape == Shape::Trapezoid ? "trapezoid" : "smooth";
        const RampProfile profile{{(lesson_line::right - lesson_line::left).Norm() * 1000.0, v}, shape};
        std::cout << "PROFILE=" << name << " CRUISE_MM_S=" << v << " LINE_S=" << profile.duration() << std::endl;
        std::vector<TipSample> samples;
        // 两次共用相同的模型、逆解、10 ms 路点间隔和 Action，只替换距离函数。
        lesson_line::run_model_line(node, "/jixiebi/learning/sampled_follow_joint_trajectory",
            [&](const KDL::Chain &chain, const KDL::JntArray &current, double &entry_s) {
                return lesson_line::make_timed_line(chain, current, profile, entry_s, 0.01);
            },
            [&](double t, const KDL::Vector &tip) { samples.push_back({t, tip.x() * 1000.0}); });
        write_reference(folder / (name + "_reference.csv"), profile);
        write_feedback(folder / (name + "_feedback.csv"), samples);
    }
    std::cout << "CURVES_DIR=" << fs::absolute(folder) << '\n';
}
}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        compare(std::make_shared<rclcpp::Node>("lesson23_compare_ramps"));
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
