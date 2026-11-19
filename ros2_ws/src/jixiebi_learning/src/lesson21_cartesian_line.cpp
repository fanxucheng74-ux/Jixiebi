#include "lesson_line_run.hpp"

namespace {
using namespace lesson_line;

Trajectory make_line(const KDL::Chain &chain, KDL::JntArray previous, int segments, double &entry_s)
{
    const auto geometry = lesson_ik::read_geometry(chain);
    Trajectory path;
    path.joint_names.assign(lesson_ik::names.begin(), lesson_ik::names.end());
    Point start;
    for (unsigned j = 0; j < previous.rows(); ++j) start.positions.push_back(previous(j));
    path.points.push_back(start);  // 第 0 秒用刚才读取的模型姿态。

    double time = 0.0;
    for (int i = 0; i <= segments; ++i) {
        const double u = static_cast<double>(i) / segments;
        const auto xyz = left + (right - left) * u;
        const auto q = lesson_ik::solve(geometry, xyz, pitch, previous);
        lesson_ik::check_solution(chain, q, xyz, pitch);  // 每个路点先回算，整条都通过才提交。

        double max_change = 0.0;
        for (size_t j = 0; j < q.size(); ++j) {
            max_change = std::max(max_change, std::abs(q[j] - previous(j)));
            previous(j) = q[j];  // 下一点从相邻解出发，减少肩肘换支的可能。
        }
        if (i == 0 && max_change < 1e-8) continue;  // 已在左端，沿用首点。
        // 原播放器逐段三次插值，峰速是平均速度的 1.5 倍。横移默认分配 4 s。
        time += std::max(i == 0 ? 1.0 : 4.0 / segments, 1.5 * max_change);
        Point next;
        next.positions.assign(q.begin(), q.end());
        next.time_from_start = rclcpp::Duration::from_seconds(time);
        path.points.push_back(next);
        if (i == 0) entry_s = lesson_trajectory::seconds_at(next);
    }
    const auto reason = lesson_trajectory::check_preview(path);
    if (!reason.empty()) throw std::runtime_error(reason);
    return path;
}

void run(const rclcpp::Node::SharedPtr &node)
{
    const auto segments = node->declare_parameter<int>("segments", 12);
    if (segments < 1 || segments > 40) throw std::invalid_argument("segments 须为 1～40 的整数");
    std::cout << "SEGMENTS=" << segments << std::endl;
    run_model_line(node, "/jixiebi/learning/follow_joint_trajectory",
        [segments](const KDL::Chain &chain, const KDL::JntArray &current, double &entry_s) {
            return make_line(chain, current, segments, entry_s);
        });
}
}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        run(std::make_shared<rclcpp::Node>("lesson21_cartesian_line"));
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
