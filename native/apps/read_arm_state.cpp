#include "arm_state.hpp"

#include <charconv>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void print_usage()
{
    std::cout << "顺序查询000～005一次，不发送运动指令\n"
                 "用法: read_arm_state --port /dev/ttyUSB0 [--timeout-ms 500]\n"
                 "模拟时另加--pty-simulation，仅用于本机/dev/pts/数字\n";
}

int parse_timeout(const std::string &text)
{
    int value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        value < 50 || value > 5000) {
        throw std::invalid_argument("--timeout-ms必须为50～5000的完整十进制整数");
    }
    return value;
}

const char *status_text(jixiebi::SampleStatus status)
{
    switch (status) {
    case jixiebi::SampleStatus::Ok:
        return "OK";
    case jixiebi::SampleStatus::Failed:
        return "FAILED";
    default:
        return "NOT_QUERIED";
    }
}

}  // namespace

int main(int argc, char *argv[])
{
    if (argc == 1) {
        print_usage();
        return EXIT_FAILURE;
    }
    try {
        std::string port;
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
            if ((option != "--port" && option != "--timeout-ms") || ++index >= argc) {
                throw std::invalid_argument("未知参数或缺少对应值: " + option);
            }
            if (option == "--port") {
                port = argv[index];
            } else {
                timeout_ms = parse_timeout(argv[index]);
            }
        }
        if (port.empty()) {
            throw std::invalid_argument("必须明确给出--port");
        }

        // 只创建一个串口对象，锁保持到整个批次打印并退出作用域之后。
        jixiebi::PositionReader reader(port, pty_simulation);
        // 函数接收reader的引用，复用同一连接；auto推导结果类型为ArmStateBatch。
        const auto batch = jixiebi::read_arm_state(
            reader, std::chrono::milliseconds(timeout_ms));

        std::cout << "SOURCE=" << (pty_simulation ? "PTY_SIMULATION" : "SERIAL_TTY")
                  << "\nSIMULTANEOUS=false\n" << std::fixed << std::setprecision(3);
        for (const auto &sample : batch.samples) {
            std::cout << "ID=" << std::setfill('0') << std::setw(3)
                      << static_cast<unsigned int>(sample.servo_id) << std::setfill(' ')
                      << " STATUS=" << status_text(sample.status) << " P=";
            if (sample.status == jixiebi::SampleStatus::Ok) {
                std::cout << sample.position_p;
            } else {
                std::cout << "--";  // 失败/未查询不能把初值0当成有效位置。
            }
            std::cout << " ELAPSED_MS=" << sample.elapsed_ms
                      << " FINISHED_OFFSET_MS=" << sample.finished_offset_ms;
            if (!sample.error.empty()) {
                std::cout << " ERROR=" << sample.error;
            }
            std::cout << '\n';
        }
        std::cout << "BATCH_STATUS=" << (batch.complete ? "OK" : "INCOMPLETE")
                  << "\nVALID_COUNT=" << batch.valid_count
                  << "\nBATCH_ELAPSED_MS=" << batch.elapsed_ms << '\n';
        return batch.complete && std::cout.good() ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception &error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
