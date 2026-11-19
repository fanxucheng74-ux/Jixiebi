#include "lesson_model.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"

#include <iomanip>

void show_position(const rclcpp::Node::SharedPtr &node)
{
    const auto chain = lesson_model::read_chain(node);
    const auto state = lesson_model::read_state(node);

    std::cout << std::fixed << std::setprecision(6);
    const auto q = lesson_model::angles_in_chain_order(chain, state);
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::Frame base_to_tip;
    // 逐段复合 URDF 中的安装变换和关节转动。结果 p 是工具原点在底座系中的坐标，单位 m。
    if (fk.JntToCart(q, base_to_tip) < 0) throw std::runtime_error("正运动学计算失败");
    const auto &p = base_to_tip.p;
    std::cout << std::setprecision(3) << "base_link -> tool_tip  XYZ_MM=["
              << p.x() * 1000.0 << ", " << p.y() * 1000.0 << ", " << p.z() * 1000.0 << "]\n";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        // 一次调用只读一个模型和一帧状态，打印后返回提示符，便于在同一终端比较两次。
        show_position(std::make_shared<rclcpp::Node>("lesson19_tool_position"));
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
