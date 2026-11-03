"""离线验证名义关节状态桥；不连接 ROS、串口或机械臂。"""

from datetime import datetime, timedelta, timezone
import math
import unittest

from arm_project.nominal_state_bridge import NominalStateBridge


BOOT = "a3c01061-3e90-4c40-a770-bade891e01cd"
SESSION = "14dd54ab4e3e474c868897a45ac4358a"
WALL_START = datetime(2026, 11, 3, 5, 30, 22, tzinfo=timezone.utc)


def sample(batch=1, offset_s=0):
    """构造与实际ROS原始状态形状一致的六轴顺序采样数据。"""
    pulses = (1600, 1400, 1700, 1300, 1504, 1514)
    servos = []
    for servo_id, pulse in enumerate(pulses):
        wall = WALL_START + timedelta(seconds=offset_s, milliseconds=50 * servo_id)
        servos.append(
            {
                "servo_id": servo_id,
                "status": "OK",
                "position_p": pulse,
                "received_at": wall.isoformat(),
                "received_monotonic_s": 1000.0 + offset_s + 0.05 * servo_id,
                "age_ms_at_finish": 260 - 50 * servo_id,
            }
        )
    return {
        "schema": "jixiebi.raw_state.v1",
        "session_id": SESSION,
        "batch_index": batch,
        "state": {
            "source": "HARDWARE",
            "status": "OK",
            "position_unit": "protocol_P",
            "simultaneous": False,
            "timestamp_origin": "host_receive",
            "host_boot_id": BOOT,
            "started_at": (WALL_START + timedelta(seconds=offset_s)).isoformat(),
            "finished_at": (WALL_START + timedelta(seconds=offset_s, milliseconds=260)).isoformat(),
            "finished_monotonic_s": 1000.26 + offset_s,
            "duration_ms": 260.0,
            "error": None,
            "servos": servos,
        },
    }


class NominalStateBridgeTests(unittest.TestCase):
    def setUp(self):
        self.bridge = NominalStateBridge(BOOT, max_age_s=3.0)

    def convert(self, payload, *, now=1000.5, wall=None, host_boot_id=BOOT):
        # 合成墙钟与单调时钟来自同一假定采样，wall可覆盖以模拟系统校时。
        if wall is None:
            wall = WALL_START.timestamp() + (now - 1000.0)
        return self.bridge.accept(
            payload, now_monotonic=now, now_wall=wall, host_boot_id=host_boot_id
        )

    def test_expected_four_nominal_angles_with_individual_receive_times(self):
        result = self.convert(sample())
        self.assertEqual(len(result), 4)
        self.assertEqual(
            [entry["name"] for entry in result],
            ["base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch"],
        )
        # 厂家约定与 URDF 方向不同，分别为 -yaw、shoulder、-elbow、wrist。
        expected_angles = (13.5, 76.5, -27.0, -27.0)
        for index, degrees in enumerate(expected_angles):
            with self.subTest(joint=index):
                self.assertAlmostEqual(result[index]["position_rad"], math.radians(degrees))
                instant = WALL_START + timedelta(milliseconds=50 * index)
                self.assertEqual(result[index]["stamp_sec"], int(instant.timestamp()))
                self.assertEqual(result[index]["stamp_nanosec"], instant.microsecond * 1000)

    def test_reject_incomplete_invalid_identity_and_semantics(self):
        variants = []
        for field, value in (
            ("source", "SIMULATION"),
            ("status", "INCOMPLETE"),
            ("error", "query timeout"),
            ("position_unit", "rad"),
            ("simultaneous", True),
            ("timestamp_origin", "device"),
            ("host_boot_id", "another-boot"),
        ):
            item = sample()
            item["state"][field] = value
            variants.append((field, item))
        item = sample()
        del item["state"]["host_boot_id"]
        variants.append(("missing_boot_id", item))
        item = sample()
        item["schema"] = "unknown"
        variants.append(("schema", item))
        item = sample()
        item["state"]["servos"].pop()
        variants.append(("missing_servo", item))
        item = sample()
        item["state"]["servos"][5]["servo_id"] = 0
        variants.append(("duplicate_servo_id", item))
        item = sample()
        item["state"]["servos"][4]["status"] = "READ_FAILED"
        variants.append(("invalid_nonmodel_servo", item))
        for label, item in variants:
            with self.subTest(label=label), self.assertRaises(ValueError):
                self.convert(item)
        # 一条无效报文不能占用会话和序号，随后的正常状态仍可接收。
        self.assertEqual(len(self.convert(sample())), 4)

    def test_reject_invalid_pulses_and_time_values(self):
        mutations = [
            ("position_p", None),
            ("position_p", True),
            ("position_p", 499),
            ("position_p", 2501),
            ("position_p", 1500.0),
            ("received_monotonic_s", float("nan")),
            ("received_monotonic_s", float("inf")),
            ("received_monotonic_s", 1002.0),
            ("received_at", "not-an-iso-date"),
            ("received_at", "2026-11-03T05:30:22"),
        ]
        for field, value in mutations:
            item = sample()
            item["state"]["servos"][2][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.convert(item)
        old = sample()
        with self.assertRaises(ValueError):
            self.convert(old, now=1004.0)
        with self.assertRaises(ValueError):
            self.convert(sample(), host_boot_id="different-boot")
        self.assertEqual(len(self.convert(sample())), 4)

    def test_reject_nonincreasing_sampling_order(self):
        for field, replacement in (
            ("received_monotonic_s", 999.99),
            ("received_at", (WALL_START - timedelta(seconds=1)).isoformat()),
        ):
            item = sample()
            item["state"]["servos"][3][field] = replacement
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.convert(item)

    def test_reject_ordered_but_inconsistent_wall_clock_and_invalid_now(self):
        # 各轴墙钟依然有序，但整体比单调时钟推算的采样时刻晚了 2 秒。
        inconsistent = sample()
        for index, item in enumerate(inconsistent["state"]["servos"]):
            wall = WALL_START + timedelta(seconds=2, milliseconds=50 * index)
            item["received_at"] = wall.isoformat()
        with self.assertRaises(ValueError):
            self.convert(inconsistent)

        # 当前墙钟不可用或大幅回拨时，不将未来/不一致时间戳发给 ROS。
        for wall in (float("nan"), float("inf"), "bad", WALL_START.timestamp() - 2):
            with self.subTest(wall=wall), self.assertRaises(ValueError):
                self.convert(sample(), wall=wall)
        self.assertEqual(len(self.convert(sample())), 4)

    def test_duplicate_older_and_different_session_are_rejected_without_state_change(self):
        self.convert(sample(batch=10))
        for batch in (10, 9):
            with self.subTest(batch=batch), self.assertRaises(ValueError):
                self.convert(sample(batch=batch))
        other = sample(batch=11, offset_s=1)
        other["session_id"] = "6d43d02a57d24c17a9c350d1537fc196"
        with self.assertRaises(ValueError):
            self.convert(other, now=1001.5)
        # 拒绝其他会话后，不影响当前会话继续增长。
        self.assertEqual(len(self.convert(sample(batch=11, offset_s=1), now=1001.5)), 4)

    def test_increasing_batch_with_replayed_timestamps_is_rejected(self):
        self.convert(sample(batch=1))
        with self.assertRaises(ValueError):
            self.convert(sample(batch=2))
        self.assertEqual(len(self.convert(sample(batch=2, offset_s=1), now=1001.5)), 4)

    def test_invalid_batch_does_not_advance_sequence(self):
        self.convert(sample(batch=1))
        invalid = sample(batch=2, offset_s=1)
        invalid["state"]["servos"][0]["position_p"] = None
        with self.assertRaises(ValueError):
            self.convert(invalid, now=1001.5)
        self.assertEqual(len(self.convert(sample(batch=2, offset_s=1), now=1001.5)), 4)


if __name__ == "__main__":
    unittest.main()
