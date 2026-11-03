"""PRTV离线解析；手册示例不是当前固件或供电的实测证据。"""

import unittest

from arm_project.servo_protocol import FrameReceiver, make_query, parse_reply, validate_read_query


class TelemetryTests(unittest.TestCase):
    def test_observed_raw_format_keeps_leading_zeroes_without_scaling(self):
        decoded = parse_reply(b"#000@T090V082!", 0, "telemetry")
        self.assertEqual(
            decoded, {"temperature_raw": "090", "voltage_raw": "082", "units_verified": False}
        )
        self.assertNotIn("temperature_c", decoded)
        self.assertNotIn("voltage_v", decoded)
        receiver = FrameReceiver()
        self.assertEqual(receiver.feed(bytes.fromhex("23")), [])
        frames = receiver.feed(bytes.fromhex("303030405430393056303832210d0a"))
        self.assertEqual(frames, [b"#000@T090V082!"])
        self.assertEqual(parse_reply(frames[0], 0, "telemetry"), decoded)
        self.assertEqual(
            parse_reply(b"#005@T000V999!", 5, "telemetry"),
            {"temperature_raw": "000", "voltage_raw": "999", "units_verified": False},
        )

    def test_observed_raw_format_remains_strict(self):
        invalid = (
            b"#001@T090V082!",
            b"#255@T090V082!",
            b"#000@T90V082!",
            b"#000@T090V82!",
            b"#000@T0900V082!",
            b"#000@T090V0820!",
            b"#000@T09xV082!",
            b"#000@T090V08x!",
            b"#000@T-90V082!",
            b"#000@T9.0V8.2!",
            b"#000@ T090V082!",
            b"#000@T090V082!!",
            b"#000@T090V082!\r\n",
            b"#000@T090v082!",
            b"#000@t090V082!",
        )
        for frame in invalid:
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                parse_reply(frame, 0, "telemetry")
        for field in ("version", "id", "mode", "position"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                parse_reply(b"#000@T090V082!", 0, field)

    def test_query_allowlist_and_all_six_addresses(self):
        for servo_id in range(6):
            with self.subTest(servo_id=servo_id):
                command = make_query(servo_id, "telemetry")
                self.assertEqual(command, ("#%03dPRTV!" % servo_id).encode("ascii"))
                self.assertEqual(validate_read_query(command), (servo_id, "telemetry"))

    def test_both_documented_reply_examples(self):
        # esp32_manual.txt:878 与971的示例精度不同，明确兼容两者。
        self.assertEqual(
            parse_reply(b"#000T28.1V7.4!", 0, "telemetry"),
            {"temperature_c": 28.1, "voltage_v": 7.4},
        )
        self.assertEqual(
            parse_reply(b"#000T25V07!", 0, "telemetry"), {"temperature_c": 25.0, "voltage_v": 7.0}
        )
        self.assertEqual(
            parse_reply(b"#005T025.0V07.4!", 5, "telemetry"),
            {"temperature_c": 25.0, "voltage_v": 7.4},
        )

    def test_logical_decimal_bounds_are_not_safe_operating_limits(self):
        # 这些是语法检查数据，不是厂家量程或可安全运行参数。
        cases = (
            (b"#000T-12.3V0!", -12.3, 0.0),
            (b"#000T120V3.2!", 120.0, 3.2),
            (b"#000T25V12.0!", 25.0, 12.0),
            (b"#000T999.9V999.9!", 999.9, 999.9),
            (b"#000T-999.9V0.0!", -999.9, 0.0),
        )
        for frame, temperature, voltage in cases:
            with self.subTest(frame=frame):
                self.assertEqual(
                    parse_reply(frame, 0, "telemetry"),
                    {"temperature_c": temperature, "voltage_v": voltage},
                )

    def test_arbitrary_splits_noise_and_multiple_frames(self):
        stream = b"noise\r\n#000T28.1V7.4!\r\n#005T25V07!"
        expected = [b"#000T28.1V7.4!", b"#005T25V07!"]
        for split in range(len(stream) + 1):
            receiver = FrameReceiver()
            with self.subTest(split=split):
                frames = receiver.feed(stream[:split]) + receiver.feed(stream[split:])
                self.assertEqual(frames, expected)
                self.assertEqual(parse_reply(frames[0], 0, "telemetry")["voltage_v"], 7.4)
                self.assertEqual(parse_reply(frames[1], 5, "telemetry")["temperature_c"], 25.0)

    def test_echo_wrong_reply_kind_and_wrong_address_are_rejected(self):
        invalid = (
            b"#000PRTV!",
            b"#000PVER!",
            b"#000PRAD!",
            b"#000PID!",
            b"#000P1513!",
            b"#000MOD1!",
            b"#000P!",
            b"#000PV0.97!",
            b"#001T28.1V7.4!",
            b"#006T28.1V7.4!",
            b"#255T28.1V7.4!",
            b"#00xT28.1V7.4!",
            b"#00T28.1V7.4!",
        )
        for frame in invalid:
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                parse_reply(frame, 0, "telemetry")
        for field in ("position", "id", "mode", "version"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                parse_reply(b"#000T28.1V7.4!", 0, field)

    def test_bad_numeric_tokens_and_units_are_rejected(self):
        invalid = (
            b"#000TNaNV7.4!",
            b"#000TinfV7.4!",
            b"#000T25VNaN!",
            b"#000T2e1V7.4!",
            b"#000T25V7e0!",
            b"#000T25V-7.4!",
            b"#000T+25V7.4!",
            b"#000T25V+7.4!",
            b"#000T--25V7.4!",
            b"#000T1000V7.4!",
            b"#000T-1000V7.4!",
            b"#000T25V1000!",
            b"#000T25.01V7.4!",
            b"#000T25V7.40!",
            b"#000T25.V7.4!",
            b"#000T.1V7.4!",
            b"#000T25V.4!",
            b"#000T25V7.!",
            b"#000T25V!",
            b"#000TV7.4!",
            b"#000T-V7.4!",
            b"#000T25 V7.4!",
            b"#000T25V 7.4!",
            b"#000T25V7.4\t!",
            b"#000T25.0.0V7.4!",
            b"#000T25V7.4V8.4!",
            b"#000T25T26V7.4!",
            b"#000t25V7.4!",
            b"#000T25v7.4!",
            b"#000T25C7.4!",
            b"#000T25V\xff!",
            b"#000T25V7\x00!",
            b"#000T25V7.4!!",
            b"#000T25V7.4!\r\n",
            b"#000T25V7.4",
            b"",
            None,
            "#000T25V7.4!",
            bytearray(b"#000T25V7.4!"),
        )
        for frame in invalid:
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                parse_reply(frame, 0, "telemetry")

    def test_no_telemetry_configuration_or_broadcast_commands(self):
        for command in (
            b"#255PRTV!",
            b"#006PRTV!",
            b"#000PSTB!",
            b"#000PSTB=60!",
            b"#000PRTV=1!",
            b"#000PRTV!\r\n",
            b"#000PRTV!#001PRTV!",
            b"#000PRTV ",
            "#000PRTV!",
        ):
            with self.subTest(command=command), self.assertRaises(ValueError):
                validate_read_query(command)
        for servo_id in (-1, 6, 255, True, False, 0.0, "0", None):
            with self.subTest(servo_id=servo_id):
                with self.assertRaises(ValueError):
                    make_query(servo_id, "telemetry")
                with self.assertRaises(ValueError):
                    parse_reply(b"#000T25V7.4!", servo_id, "telemetry")


if __name__ == "__main__":
    unittest.main()
