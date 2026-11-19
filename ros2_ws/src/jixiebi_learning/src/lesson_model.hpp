#pragma once

#include "kdl/jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace lesson_model {
using State = sensor_msgs::msg::JointState;
using namespace std::chrono_literals;

inline KDL::Chain read_chain(const rclcpp::Node::SharedPtr &node)
{
    // 状态发布器保留一份已展开的 URDF。晚启动的读者用 transient_local 也能收到。
    std_msgs::msg::String urdf;
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    if (!rclcpp::wait_for_message(urdf, node, "/jixiebi/learning/follow/robot_description", 5s, qos)) {
        throw std::runtime_error("未收到模型描述，请确认终端 A 已启动 checked_action 模型");
    }
    KDL::Tree tree;
    KDL::Chain chain;
    if (!kdl_parser::treeFromString(urdf.data, tree)) throw std::runtime_error("URDF 解析失败");
    if (!tree.getChain("base_link", "tool_tip", chain)) {
        throw std::runtime_error("模型中找不到 base_link 到 tool_tip 的链");
    }
    return chain;
}

inline KDL::JntArray angles_in_chain_order(const KDL::Chain &chain, const State &state)
{
    if (state.name.size() != state.position.size()) throw std::runtime_error("关节名和角度数量不符");
    KDL::JntArray q(chain.getNrOfJoints());
    unsigned int column = 0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const auto &joint = chain.getSegment(i).getJoint();
        // 固定工具段有长度，但不需要关节角。它保留在 chain 中参与最终计算。
        if (joint.getType() == KDL::Joint::Fixed) continue;
        const auto &name = joint.getName();
        const auto it = std::find(state.name.begin(), state.name.end(), name);
        if (it == state.name.end() || std::find(std::next(it), state.name.end(), name) != state.name.end()) {
            throw std::runtime_error("关节缺失或重复：" + name);
        }
        // 按模型链的顺序填写 q，消息中同一关节可以出现在其他列。
        const double rad = state.position[std::distance(state.name.begin(), it)];
        if (!std::isfinite(rad)) throw std::runtime_error("关节角不是有限数：" + name);
        q(column++) = rad;
        std::cout << name << '=' << rad << " rad\n";
    }
    return q;
}

// 读取播放器此刻的四轴模型角，两课共享关节名和时间戳检查。
inline State read_state(const rclcpp::Node::SharedPtr &node)
{
    State state;
    if (!rclcpp::wait_for_message(state, node, "/jixiebi/learning/follow/joint_states", 3s)) {
        throw std::runtime_error("未收到模型的四轴状态");
    }
    const auto age = node->now() - rclcpp::Time(state.header.stamp);
    if (age.nanoseconds() < 0 || age.seconds() > 0.5) throw std::runtime_error("模型状态时间戳超前或过期");
    return state;
}
}  // namespace lesson_model
