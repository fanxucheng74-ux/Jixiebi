#ifndef JIXIEBI_POSITION_READER_HPP
#define JIXIEBI_POSITION_READER_HPP

#include <chrono>
#include <cstdint>
#include <string>

namespace jixiebi {

// Linux用户态串口对象；本课只有位置查询，没有发送任意命令或运动的接口。
// 一个对象只由一个调用者使用，不提供线程同步；后续ROS节点复用它而不另开串口。
class PositionReader {
public:
    // 构造成功才拥有已配置的设备；失败抛异常并释放已获得的文件描述符/锁。
    // pty_simulation仅供本机/dev/pts/数字测试，不能把模拟验收写成硬件通过。
    explicit PositionReader(const std::string &port, bool pty_simulation = false);
    ~PositionReader();

    // 文件描述符不能按普通整数复制，否则两个对象会关闭同一个设备。
    PositionReader(const PositionReader &) = delete;
    PositionReader &operator=(const PositionReader &) = delete;

    // 一次查询只发送一次PRAD；总期限覆盖写入及接收，不因坏帧而重新计时。
    // 返回协议P值，不是角度/毫米。超时或断开抛异常，不自动重发。
    std::uint16_t read_position(std::uint8_t servo_id,
                                std::chrono::milliseconds timeout);

private:
    int fd_ = -1;  // Linux文件描述符；-1表示尚未拥有设备。
};

}  // namespace jixiebi

#endif  // JIXIEBI_POSITION_READER_HPP
