#include "position_reader.hpp"

#include "servo_codec.h"
#include "servo_receiver.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <system_error>
#include <termios.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;  // 不受系统校时影响，用于本机相对期限。
constexpr std::size_t kMaxReceivedBytes = 4096U;  // 本次查询的主机内存/处理策略。

[[noreturn]] void throw_system_error(const char *operation)
{
    const int saved_errno = errno;
    throw std::system_error(saved_errno, std::generic_category(), operation);
}

bool is_pty_path(const std::string &path)
{
    const std::string prefix = "/dev/pts/";
    return path.compare(0, prefix.size(), prefix) == 0 && path.size() > prefix.size() &&
           std::all_of(path.begin() + static_cast<std::string::difference_type>(prefix.size()),
                       path.end(), [](char ch) { return ch >= '0' && ch <= '9'; });
}

void configure_port(int fd, bool pty_simulation)
{
    termios options{};
    if (::tcgetattr(fd, &options) == -1) {
        throw_system_error("tcgetattr: 不是可配置的TTY");
    }
    // raw模式关闭行缓冲/本地回显/控制字符转换，让协议按原始字节传递。
    ::cfmakeraw(&options);
    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | HUPCL);
    options.c_cflag |= CS8 | CLOCAL | CREAD;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
    if (::cfsetispeed(&options, B115200) == -1 ||
        ::cfsetospeed(&options, B115200) == -1) {
        throw_system_error("cfsetspeed");
    }
    if (::tcsetattr(fd, TCSANOW, &options) == -1) {
        throw_system_error("tcsetattr");
    }

    // PTY没有真实DTR/RTS电平，必须显式区分模拟。真实设备失败则报错，不静默略过。
    if (!pty_simulation) {
        int modem_bits = 0;
        if (::ioctl(fd, TIOCMGET, &modem_bits) == -1) {
            throw_system_error("TIOCMGET");
        }
        modem_bits &= ~(TIOCM_DTR | TIOCM_RTS);
        if (::ioctl(fd, TIOCMSET, &modem_bits) == -1) {
            throw_system_error("TIOCMSET: 清除DTR/RTS失败");
        }
    }
}

void wait_ready(int fd, short events, Clock::time_point deadline)
{
    while (true) {
        const auto now = Clock::now();
        if (now >= deadline) {
            throw std::runtime_error("query timeout: 未在总期限内收到匹配回复");
        }
        // 向上取整，避免剩余不足1ms时被截断成0；每次被信号打断后重算剩余时间。
        const auto left_ms = std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
        pollfd item{fd, events, 0};
        const int result = ::poll(&item, 1, static_cast<int>(left_ms.count()));
        if (result == -1) {
            if (errno == EINTR) {
                continue;
            }
            throw_system_error("poll");
        }
        if (result == 0) {
            continue;
        }
        // poll醒来后可能刚好过期或被延迟调度，不能仅凭“曾经就绪”放行迟到回复。
        if (Clock::now() >= deadline) {
            throw std::runtime_error("query timeout: 就绪事件到达时已过总期限");
        }
        if ((item.revents & POLLNVAL) != 0) {
            throw std::runtime_error("poll: 文件描述符失效");
        }
        // 读侧可能同时报告POLLIN/HUP，先读完已到达字节；没有可读数据才报告断开。
        if ((item.revents & events) != 0) {
            return;
        }
        if ((item.revents & (POLLERR | POLLHUP)) != 0) {
            throw std::runtime_error("serial disconnected: TTY错误或设备断开");
        }
    }
}

void write_query(int fd, const char *query, std::size_t length, Clock::time_point deadline)
{
    std::size_t sent = 0U;
    while (sent < length) {
        wait_ready(fd, POLLOUT, deadline);
        const ssize_t count = ::write(fd, query + sent, length - sent);
        if (count > 0) {
            sent += static_cast<std::size_t>(count);
        } else if (count == -1 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        } else if (count == -1) {
            throw_system_error("write");
        } else {
            throw std::runtime_error("write: 未能继续写入查询");
        }
    }
    // 这里只补写未写入的尾部，不重新发送整条查询；write完成也不表示舵机已回复。
}

}  // namespace

namespace jixiebi {

PositionReader::PositionReader(const std::string &port, bool pty_simulation)
{
    if (port.empty() || (pty_simulation && !is_pty_path(port))) {
        throw std::invalid_argument("端口为空，或PTY模拟路径不是/dev/pts/数字");
    }
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ == -1) {
        throw_system_error("open");
    }
    try {
        // 与旧pySerial exclusive=True使用相同的flock协作锁；先拿锁再改串口配置。
        // 它不能阻止不遵守锁的其他程序，仍须停止原ROS/Python串口所有者。
        if (::flock(fd_, LOCK_EX | LOCK_NB) == -1) {
            throw_system_error("flock: 串口已占用");
        }
        configure_port(fd_, pty_simulation);
    } catch (...) {
        // 构造函数失败时析构函数不会执行，所以在这里关闭，连同锁一起释放。
        ::close(fd_);
        fd_ = -1;
        throw;
    }
}

PositionReader::~PositionReader()
{
    if (fd_ >= 0) {
        // Linux上close被EINTR打断也不重试，避免误关已复用的描述符；关闭释放flock。
        ::close(fd_);
    }
}

std::uint16_t PositionReader::read_position(std::uint8_t servo_id,
                                          std::chrono::milliseconds timeout)
{
    if (timeout.count() < 50 || timeout.count() > 5000) {
        throw std::invalid_argument("查询期限必须为50～5000毫秒");
    }
    char query[SERVO_POSITION_QUERY_CAPACITY]{};
    if (servo_make_position_query(servo_id, query, sizeof(query)) != SERVO_CODEC_OK) {
        throw std::invalid_argument("舵机编号必须为0～5");
    }
    servo_frame_receiver_t receiver{};
    if (!servo_frame_receiver_init(&receiver)) {
        throw std::runtime_error("接收器初始化失败");
    }
    // 只在发查询前丢弃已排队旧输入；发出后flush会误删快速回复。该操作不能识别在途迟到帧。
    if (::tcflush(fd_, TCIFLUSH) == -1) {
        throw_system_error("tcflush");
    }
    const auto deadline = Clock::now() + timeout;
    write_query(fd_, query, SERVO_POSITION_QUERY_LENGTH, deadline);  // 不发送C字符串的\0。
    std::cerr << "TX: " << query << '\n';

    std::size_t received_bytes = 0U;
    while (true) {
        wait_ready(fd_, POLLIN, deadline);
        char chunk[128]{};
        const ssize_t count = ::read(fd_, chunk, sizeof(chunk));
        if (count == -1 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        }
        if (count == -1) {
            throw_system_error("read");
        }
        if (count == 0) {
            throw std::runtime_error("serial disconnected: read返回0");
        }
        received_bytes += static_cast<std::size_t>(count);
        if (received_bytes > kMaxReceivedBytes) {
            throw std::runtime_error("查询接收量超过4096字节，终止本次事务");
        }
        // count是read实际返回的字节数，不能用strlen；接收器在每次read之间保持半帧。
        for (ssize_t index = 0; index < count; ++index) {
            const auto event = servo_frame_receiver_push(
                &receiver, static_cast<std::uint8_t>(chunk[index]));
            if (event == SERVO_RX_INVALID_ARGUMENT) {
                throw std::runtime_error("接收器状态非法");
            }
            if (event != SERVO_RX_FRAME_READY) {
                continue;
            }
            // 回显可能与回复粘连；只跳过完全相同的9字节查询，不将它当成位置。
            if (receiver.used == SERVO_POSITION_QUERY_LENGTH &&
                std::memcmp(receiver.buffer, query, receiver.used) == 0) {
                continue;
            }
            std::uint16_t position_p = 0U;
            const auto status = servo_parse_position_reply(
                receiver.buffer, receiver.used, servo_id, &position_p);
            if (status == SERVO_CODEC_OK) {
                if (Clock::now() >= deadline) {
                    throw std::runtime_error("query timeout: 解析完成时已过总期限");
                }
                std::cerr << "RX: ";
                std::cerr.write(receiver.buffer, static_cast<std::streamsize>(receiver.used));
                std::cerr << '\n';
                return position_p;  // 下一次push会复用缓冲区，因此必须在这里立即处理。
            }
            std::cerr << "忽略候选帧，解析状态=" << static_cast<int>(status) << '\n';
        }
    }
}

}  // namespace jixiebi
