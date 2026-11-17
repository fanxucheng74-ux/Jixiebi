#include "arm_state.hpp"

#include <exception>
#include <stdexcept>

namespace jixiebi {

ArmStateBatch read_arm_state(PositionReader &reader, std::chrono::milliseconds timeout)
{
    if (timeout.count() < 50 || timeout.count() > 5000) {
        throw std::invalid_argument("每轴查询期限必须为50～5000毫秒");
    }
    ArmStateBatch batch{};
    // 先给所有行登记编号；即使中途失败，未查询的行也不会都误显示为000。
    for (std::size_t index = 0U; index < batch.samples.size(); ++index) {
        batch.samples[index].servo_id = static_cast<std::uint8_t>(index);
    }

    using Clock = std::chrono::steady_clock;
    const auto batch_started = Clock::now();
    for (auto &sample : batch.samples) {
        const auto query_started = Clock::now();
        try {
            // 复用上一课已经实机读通的事务：写PRAD→流接收→完整解析→返回协议P。
            sample.position_p = reader.read_position(sample.servo_id, timeout);
            sample.status = SampleStatus::Ok;
            ++batch.valid_count;
        } catch (const std::exception &error) {
            sample.status = SampleStatus::Failed;
            sample.error = error.what();
        }
        const auto query_finished = Clock::now();
        sample.elapsed_ms =
            std::chrono::duration<double, std::milli>(query_finished - query_started).count();
        sample.finished_offset_ms =
            std::chrono::duration<double, std::milli>(query_finished - batch_started).count();
        if (sample.status == SampleStatus::Failed) {
            // 失败不是0P，也不自动补查；其余行保留NotQueried，让调用者识别不完整批次。
            batch.elapsed_ms = sample.finished_offset_ms;
            return batch;
        }
    }
    batch.complete = true;
    batch.elapsed_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - batch_started).count();
    return batch;
}

}  // namespace jixiebi
