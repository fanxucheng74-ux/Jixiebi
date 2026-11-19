#include "calibrated_feedback.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using jixiebi::checked_calibration;
using jixiebi::convert_feedback;

// 所有数值仅供内存单元测试，SERIAL_TTY 是待测消息字段，无串口、DDS 或实物。
jixiebi::RawBatch synthetic_batch()
{
    jixiebi::RawBatch raw;
    raw.source = "SERIAL_TTY";
    raw.complete = true;
    raw.valid_count = 6;
    raw.sequence = 1;
    raw.stamp.sec = 123;
    raw.stamp.nanosec = 456;
    raw.elapsed_ms = 6.0;
    for (std::size_t id = 0; id < raw.servos.size(); ++id) {
        auto &sample = raw.servos[id];
        sample.servo_id = static_cast<std::uint8_t>(id);
        sample.status = jixiebi_interfaces::msg::ServoStateRaw::OK;
        sample.position_p = 1600;
        sample.elapsed_ms = 1.0;
        sample.finished_offset_ms = static_cast<double>(id + 1);
    }
    return raw;
}

TEST(Calibration, RequiresTwoDistinctPairsForTheCorrectJoint)
{
    const auto missing = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(checked_calibration(1, -1, -1, missing, -1, missing));
    EXPECT_THROW(checked_calibration(1, -1, 1400, missing, -1, missing), std::invalid_argument);
    EXPECT_THROW(checked_calibration(1, 0, 1400, -1., 1500, 4.), std::invalid_argument);
    EXPECT_THROW(checked_calibration(1, 1, 1400, missing, 1500, 4.), std::invalid_argument);
    EXPECT_THROW(checked_calibration(1, 1, 1400, 1., 1500, 1.), std::invalid_argument);
    EXPECT_THROW(checked_calibration(1, 1, 1400, 1., 1401, 4.), std::invalid_argument);
    EXPECT_THROW(checked_calibration(1, 1, 499, 1., 1500, 4.), std::invalid_argument);
    EXPECT_THROW(checked_calibration(1, 1, 1400, 1., 2501, 4.), std::invalid_argument);
    EXPECT_THROW(checked_calibration(1, 1, 1400, -91., 1500, 4.), std::invalid_argument);
    EXPECT_TRUE(checked_calibration(1, 1, 2500, -90., 500, 90.));
}

TEST(Feedback, InterpolatesReversedPulseOrderAndKeepsRawStamp)
{
    auto raw = synthetic_batch();
    jixiebi::Calibrations calibrations{};
    calibrations[2] = checked_calibration(2, 2, 1700, -50., 1500, 10.);
    jixiebi::JointState output;
    const double radians_per_degree = std::acos(-1.0) / 180.0;
    ASSERT_EQ(convert_feedback(raw, calibrations, output), nullptr);
    ASSERT_EQ(output.name.size(), 1U);
    ASSERT_EQ(output.position.size(), 1U);
    EXPECT_EQ(output.name.front(), "elbow_flex");
    EXPECT_NEAR(output.position.front(), -20.0 * radians_per_degree, 1e-12);
    EXPECT_EQ(output.header.stamp, raw.stamp);
    EXPECT_TRUE(output.velocity.empty());
    EXPECT_TRUE(output.effort.empty());
    // 实测端点也允许；越出1P不做钳位或外推。
    raw.servos[2].position_p = 1500;
    ASSERT_EQ(convert_feedback(raw, calibrations, output), nullptr);
    EXPECT_NEAR(output.position.front(), 10.0 * radians_per_degree, 1e-12);
    raw.servos[2].position_p = 1700;
    ASSERT_EQ(convert_feedback(raw, calibrations, output), nullptr);
    EXPECT_NEAR(output.position.front(), -50.0 * radians_per_degree, 1e-12);
    raw.servos[2].position_p = 1701;
    EXPECT_NE(convert_feedback(raw, calibrations, output), nullptr);
}

TEST(Feedback, RejectsBadBatchesBeforeReplacingAnyJoint)
{
    auto raw = synthetic_batch();
    jixiebi::Calibrations calibrations{};
    jixiebi::JointState output;
    EXPECT_NE(convert_feedback(raw, calibrations, output), nullptr);
    calibrations[0] = checked_calibration(0, 0, 1500, 5., 1700, 55.);
    calibrations[3] = checked_calibration(3, 3, 1500, -6., 1700, 4.);
    // 最后才发现腕超界，前面已计算的底座不能单独流出。
    raw.servos[3].position_p = 1499;
    EXPECT_NE(convert_feedback(raw, calibrations, output), nullptr);
    EXPECT_TRUE(output.name.empty());

    const auto reject = [&](auto change) {
        auto bad = synthetic_batch();
        change(bad);
        EXPECT_NE(convert_feedback(bad, calibrations, output), nullptr);
    };
    reject([](auto &b) { b.source = "PTY_SIMULATION"; });
    reject([](auto &b) { b.complete = false; });
    reject([](auto &b) { b.valid_count = 5; });
    reject([](auto &b) { b.simultaneous = true; });
    reject([](auto &b) { b.stamp.sec = 0; b.stamp.nanosec = 0; });
    reject([](auto &b) { b.stamp.nanosec = 1000000000U; });
    reject([](auto &b) { b.servos[4].servo_id = 3; });
    reject([](auto &b) { b.servos[4].status = jixiebi_interfaces::msg::ServoStateRaw::FAILED; });
    reject([](auto &b) { b.servos[5].position_p = 0; });
    reject([](auto &b) { b.servos[5].error = "failed"; });
    reject([](auto &b) { b.elapsed_ms = std::numeric_limits<double>::quiet_NaN(); });
    reject([](auto &b) { b.servos[4].finished_offset_ms = 1.0; });
    reject([](auto &b) { b.servos[5].elapsed_ms = -1.0; });
    EXPECT_TRUE(output.name.empty());
}

TEST(Feedback, NominalDisplayReusesUrdfSignsWithOnlyYawRecentered)
{
    auto raw = synthetic_batch();
    raw.servos[0].position_p = 1511;
    raw.servos[1].position_p = 1500;
    raw.servos[2].position_p = 1600;
    raw.servos[3].position_p = 1400;
    jixiebi::JointState output;
    ASSERT_EQ(convert_feedback(raw, {}, output, 1511), nullptr);
    ASSERT_EQ(output.name.size(), 4U);
    ASSERT_EQ(output.position.size(), 4U);
    EXPECT_EQ(output.name.front(), "base_yaw");
    const double deg = std::acos(-1.0) / 180.0;
    EXPECT_DOUBLE_EQ(output.position[0], 0.0);
    EXPECT_NEAR(output.position[1], 90.0 * deg, 1e-12);
    EXPECT_NEAR(output.position[2], -13.5 * deg, 1e-12);
    EXPECT_NEAR(output.position[3], -13.5 * deg, 1e-12);
    raw.servos[0].position_p = 1611;
    ASSERT_EQ(convert_feedback(raw, {}, output, 1511), nullptr);
    EXPECT_NEAR(output.position[0], 13.5 * deg, 1e-12);
    // 两种显示模式都要通过同一批次检查。
    raw.complete = false;
    EXPECT_NE(convert_feedback(raw, {}, output, 1511), nullptr);
}

}  // namespace
