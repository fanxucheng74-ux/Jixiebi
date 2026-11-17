#include "position_reader.hpp"

#include <charconv>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int parse_number(const std::string &text, int minimum, int maximum)
{
    int value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
        value < minimum || value > maximum) {
        throw std::invalid_argument("参数必须是规定范围内的完整十进制整数");
    }
    return value;
}

void print_usage()
{
    std::cout << "只读取一次舵机位置，不发送运动指令\n"
                 "用法: read_position --port /dev/ttyUSB0 --id 2 [--timeout-ms 500]\n"
                 "模拟: read_position --port /dev/pts/数字 --id 2 --pty-simulation\n";
}

}  // namespace

int main(int argc, char *argv[])
{
    // 没有默认设备路径：仅运行程序或查看帮助均不会打开串口。
    if (argc == 1) {
        print_usage();
        return EXIT_FAILURE;
    }
    try {
        std::string port;
        int servo_id = -1;
        int timeout_ms = 500;
        bool pty_simulation = false;
        for (int index = 1; index < argc; ++index) {
            const std::string option = argv[index];
            if (option == "--help") {
                print_usage();
                return EXIT_SUCCESS;
            }
            if (option == "--pty-simulation") {
                pty_simulation = true;
                continue;
            }
            if (option != "--port" && option != "--id" && option != "--timeout-ms") {
                throw std::invalid_argument("未知参数: " + option);
            }
            if (++index >= argc) {
                throw std::invalid_argument("参数缺少对应值: " + option);
            }
            if (option == "--port") {
                port = argv[index];
            } else if (option == "--id") {
                // 先按int校验0～5，再转uint8_t，避免256等非法输入截断成0。
                servo_id = parse_number(argv[index], 0, 5);
            } else {
                timeout_ms = parse_number(argv[index], 50, 5000);
            }
        }
        if (port.empty() || servo_id < 0) {
            throw std::invalid_argument("必须明确给出--port和--id");
        }
        jixiebi::PositionReader reader(port, pty_simulation);
        const auto position_p = reader.read_position(
            static_cast<std::uint8_t>(servo_id), std::chrono::milliseconds(timeout_ms));
        std::cout << "SOURCE=" << (pty_simulation ? "PTY_SIMULATION" : "SERIAL_TTY")
                  << "\nSERVO_ID=" << servo_id << "\nPOSITION_P=" << position_p << '\n';
        // 离开作用域时调用析构函数关闭设备；std::exception分支也会释放资源。
        return std::cout.good() ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception &error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
