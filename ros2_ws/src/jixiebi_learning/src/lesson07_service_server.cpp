#include <exception>
#include <iostream>
#include <memory>

#include "jixiebi_interfaces/srv/get_lesson_info.hpp"
#include "rclcpp/rclcpp.hpp"

// using只是缩短生成类型的名字，不会创建节点或发送请求。
using GetLessonInfo = jixiebi_interfaces::srv::GetLessonInfo;

class LessonInfoServer : public rclcpp::Node {
public:
    LessonInfoServer() : Node("jixiebi_lesson07_server") {
        // 创建服务并保存为成员，使它在整个节点生命周期内存在。
        // lambda先登记；只有spin收到请求时，ROS才会调用下面的函数体。
        service_ = create_service<GetLessonInfo>(
            "/jixiebi/learning/get_info",
            [this](const std::shared_ptr<GetLessonInfo::Request> request,
                   std::shared_ptr<GetLessonInfo::Response> response) {
                // 这里只查静态课程资料，不读取RK、串口或用户真实学习进度。
                response->source = "STATIC_LESSON";
                response->found = request->lesson_id == 7;
                if (response->found) {
                    response->title = "ROS 2 Service请求与响应";
                    response->message = "仅返回静态课程说明，不表示学习已完成或硬件已连接。";
                } else {
                    response->title = "";
                    response->message = "本示例仅提供第7课，请查询lesson_id=7。";
                }
                RCLCPP_INFO(get_logger(), "收到课程查询：lesson_id=%u，found=%s",
                            static_cast<unsigned int>(request->lesson_id),
                            response->found ? "true" : "false");
                // 填好response后返回，ROS负责把响应发给对应客户端。
            });
        RCLCPP_INFO(get_logger(), "静态课程服务就绪：/jixiebi/learning/get_info；不连接硬件");
    }

private:
    rclcpp::Service<GetLessonInfo>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        // main不手动调用服务函数；spin等待请求并调度构造函数登记的回调。
        auto node = std::make_shared<LessonInfoServer>();
        rclcpp::spin(node);
    } catch (const std::exception &error) {
        std::cerr << "SERVER_ERROR: " << error.what() << '\n';
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
