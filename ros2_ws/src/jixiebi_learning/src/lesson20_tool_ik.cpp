#include "lesson_model.hpp"
#include "lesson_model_action.hpp"
#include "lesson_ik.hpp"

#include <iomanip>

namespace {
using namespace lesson_ik;

void run(const rclcpp::Node::SharedPtr &node)
{
    // 输入为 base_link 系下的毫米，进入 KDL 时转为米。俯仰从随底座旋转的水平正向起算。
    const KDL::Vector target(node->declare_parameter("x_mm", 0.108) / 1000.0,
                             node->declare_parameter("y_mm", 45.991) / 1000.0,
                             node->declare_parameter("z_mm", 530.987) / 1000.0);
    const double deg = node->declare_parameter("pitch_deg", 80.0);
    if (!std::isfinite(target.x()) || !std::isfinite(target.y()) || !std::isfinite(target.z()) ||
        !std::isfinite(deg) || std::abs(deg) > 180.0) throw std::invalid_argument("需要有限 XYZ 和 ±180° 内的俯仰");
    const double pitch = deg * kRadPerDegree;
    const auto chain = lesson_model::read_chain(node);
    const auto geometry = read_geometry(chain);
    using Follow = lesson_action::Follow;
    auto client = rclcpp_action::create_client<Follow>(node, "/jixiebi/learning/follow_joint_trajectory");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) throw std::runtime_error("未找到终端 A 的模型 Action");
    const auto current = lesson_model::angles_in_chain_order(chain, lesson_model::read_state(node));
    const auto q = solve(geometry, target, pitch, current);

    // 把所选解送回第十九课的 KDL 正解。比较球心位置和工具杆局部 +Y 的朝向。
    const double error_mm = check_solution(chain, q, target, pitch);
    std::cout << std::fixed << std::setprecision(6) << "IK_DEG";
    for (size_t i = 0; i < q.size(); ++i) std::cout << ' ' << names[i] << '=' << q[i] / kRadPerDegree;
    std::cout << "\nFK_ERROR_MM=" << error_mm << std::endl;

    // 取当前采样作首点，逆解作末点。原播放器用三次插值，峰值速度为平均速度的 1.5 倍。
    Follow::Goal goal;
    goal.trajectory.joint_names.assign(names.begin(), names.end());
    lesson_trajectory::Point start, finish;
    double largest_move = 0.0;
    for (size_t i = 0; i < q.size(); ++i) {
        start.positions.push_back(current(i));
        largest_move = std::max(largest_move, std::abs(q[i] - current(i)));
    }
    if (largest_move < 1e-8) { std::cout << "ALREADY_AT_TARGET\n"; return; }
    finish.positions.assign(q.begin(), q.end());
    // 第十二课模型的速度上限为 1 rad/s。至少两秒，较远的关节目标自动延长。
    finish.time_from_start.sec = static_cast<int>(std::max(2.0, std::ceil(1.5 * largest_move)));
    goal.trajectory.points = {start, finish};
    const auto reason = lesson_trajectory::check_preview(goal.trajectory);
    if (!reason.empty()) throw std::runtime_error(reason);
    lesson_action::play(node, client, goal);
}
}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        run(std::make_shared<rclcpp::Node>("lesson20_tool_ik"));
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
