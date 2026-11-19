#include <chrono>
#include <exception>
#include <iostream>
#include <memory>

#include "jixiebi_interfaces/srv/get_lesson_info.hpp"
#include "rclcpp/rclcpp.hpp"

using GetLessonInfo = jixiebi_interfaces::srv::GetLessonInfo;
using namespace std::chrono_literals;

// 返回值给main：0=业务成功，1=未找到课程，2=发现失败，3=等待响应失败。
int query_lesson(const std::shared_ptr<rclcpp::Node> &node) {
    auto client = node->create_client<GetLessonInfo>("/jixiebi/learning/get_info");
    // 5s表示5秒。发现服务和等待响应是不同阶段，分别设置期限。
    if (!client->wait_for_service(5s)) {
        RCLCPP_ERROR(node->get_logger(), "5秒内未发现课程服务；没有发送请求");
        return 2;
    }

    // 创建请求对象，填写自定义srv中---上方的字段；本课固定查询7。
    auto request = std::make_shared<GetLessonInfo::Request>();
    request->lesson_id = 7;
    // 整个程序只发送这一次。future代表稍后才可取得的响应。
    auto future = client->async_send_request(request);
    const auto outcome = rclcpp::spin_until_future_complete(node, future, 5s);
    if (outcome != rclcpp::FutureReturnCode::SUCCESS) {
        // 超时/中断只清理本地等待记录，不取消服务端执行，也不自动重发。
        client->remove_pending_request(future);
        RCLCPP_ERROR(node->get_logger(), "等待响应超时或中断；未重发，也不代表服务端已取消");
        return 3;
    }

    // 仅SUCCESS后才能get，避免在这里无期限阻塞。
    const auto response = future.get();
    std::cout << "SOURCE=" << response->source << '\n'
              << "LESSON_ID=" << static_cast<unsigned int>(request->lesson_id) << '\n'
              << "FOUND=" << (response->found ? "true" : "false") << '\n'
              << "TITLE=" << response->title << '\n'
              << "MESSAGE=" << response->message << '\n';
    return response->found ? 0 : 1;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int exit_code = 1;
    try {
        auto node = std::make_shared<rclcpp::Node>("jixiebi_lesson07_client");
        exit_code = query_lesson(node);  // main调用本课客户端逻辑，完成一次后退出。
    } catch (const std::exception &error) {
        std::cerr << "CLIENT_ERROR: " << error.what() << '\n';
    }
    rclcpp::shutdown();
    return exit_code;
}
