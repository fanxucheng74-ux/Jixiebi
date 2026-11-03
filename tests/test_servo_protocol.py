"""纯字节协议测试；不导入串口库，不连接或操作机械臂。"""

import unittest

from arm_project.servo_protocol import (
    FrameReceiver,
    make_move_command,
    make_position_query,
    make_query,
    parse_position_reply,
    parse_reply,
    validate_motion_command,
    validate_read_query,
)


class ReadQueryTests(unittest.TestCase):
    def test_all_six_queries_round_trip(self):
        for servo_id in range(6):
            with self.subTest(servo_id=servo_id):
                command = make_position_query(servo_id)
                self.assertEqual(command, ("#%03dPRAD!" % servo_id).encode("ascii"))
                self.assertEqual(validate_read_query(command), (servo_id, "position"))

    def test_all_query_fields_and_default(self):
        suffixes = {"position": b"PRAD!", "mode": b"PMOD!", "version": b"PVER!", "id": b"PID!"}
        self.assertEqual(make_query(0), b"#000PRAD!")
        for servo_id in range(6):
            for field, suffix in suffixes.items():
                with self.subTest(servo_id=servo_id, field=field):
                    command = make_query(servo_id, field)
                    self.assertEqual(command, ("#%03d" % servo_id).encode("ascii") + suffix)
                    self.assertEqual(validate_read_query(command), (servo_id, field))

    def test_invalid_query_fields(self):
        for field in ("stop", "reset", "Position", "PRAD", "", True, None, [], b"position"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                make_query(0, field)

    def test_invalid_ids_including_bool(self):
        for servo_id in (-1, 6, 254, 255, True, False, 0.0, "0", None):
            with self.subTest(servo_id=servo_id), self.assertRaises(ValueError):
                make_position_query(servo_id)

    def test_only_exact_read_allowlist_is_accepted(self):
        invalid = (
            b"#255PRAD!",
            b"#006PRAD!",
            b"#000PV!",
            b"#000PID001!",
            b"#000PMOD1!",
            b"#000PDST!",
            b"#000PULK!",
            b"$DST!",
            b"#000P1500T1000!",
            b"#00PRAD!",
            b"#00xPRAD!",
            b"#000PRAD!\r\n",
            b" #000PRAD!",
            b"#000PRAD!#001PRAD!",
            b"",
            "#000PRAD!",
            bytearray(b"#000PRAD!"),
            None,
        )
        for command in invalid:
            with self.subTest(command=command), self.assertRaises(ValueError):
                validate_read_query(command)


class PositionReplyTests(unittest.TestCase):
    def test_historical_frame_and_protocol_value_edges(self):
        self.assertEqual(parse_position_reply(b"#000P1513!", 0), 1513)
        self.assertEqual(parse_position_reply(b"#000P0500!", 0), 500)
        self.assertEqual(parse_position_reply(b"#005P2500!", 5), 2500)

    def test_wrong_id_and_unsupported_id(self):
        for frame, expected_id in ((b"#001P1500!", 0), (b"#006P1500!", 0), (b"#255P1500!", 0)):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                parse_position_reply(frame, expected_id)

    def test_expected_id_is_strict(self):
        for expected_id in (-1, 6, 255, True, False, 0.0, "0", None):
            with self.subTest(expected_id=expected_id), self.assertRaises(ValueError):
                parse_position_reply(b"#000P1500!", expected_id)

    def test_reject_echo_wrong_fields_non_digits_and_out_of_range(self):
        invalid = (
            b"#000PRAD!",
            b"#000P!",
            b"#000MOD1!",
            b"#000P1500T1000!",
            b"#000P0499!",
            b"#000P2501!",
            b"#000P9999!",
            b"#00xP1500!",
            b"#000P15x0!",
            b"#000P+500!",
            b"#000P-500!",
            b"#000P 500!",
            b"#000P\xff500!",
            b"#000P150!",
            b"#000P01500!",
            b"#000p1500!",
            b"#000P1500!\r\n",
            b"#000P1500!!",
            b"000P1500!",
            b"#000P1500",
            b"",
            "#000P1500!",
            None,
        )
        for frame in invalid:
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                parse_position_reply(frame, 0)


class QueryReplyTests(unittest.TestCase):
    def test_actual_and_legacy_mode_version_id_formats(self):
        actual_version = "ZL-ZServo_AD_CBM V2.1.16STG"
        self.assertEqual(
            parse_reply(b"#000@ " + actual_version.encode("ascii") + b"!", 0, "version"),
            actual_version,
        )
        self.assertEqual(parse_reply(b"#000PV0.97!", 0, "version"), "0.97")
        for servo_id in range(6):
            prefix = ("#%03d" % servo_id).encode("ascii")
            self.assertEqual(parse_reply(prefix + b"P!", servo_id, "id"), servo_id)
            self.assertEqual(parse_reply(prefix + b"P1513!", servo_id, "position"), 1513)
            for mode in range(1, 9):
                for tag in (b"MOD", b"PMOD"):
                    frame = prefix + tag + str(mode).encode("ascii") + b"!"
                    with self.subTest(servo_id=servo_id, mode=mode, tag=tag):
                        self.assertEqual(parse_reply(frame, servo_id, "mode"), mode)

    def test_all_query_echoes_and_cross_field_replies_are_rejected(self):
        replies = {
            "position": b"#000P1513!",
            "mode": b"#000MOD1!",
            "version": b"#000@ ZL-ZServo_AD_CBM V2.1.16STG!",
            "id": b"#000P!",
        }
        for expected in replies:
            for query in replies:
                with self.subTest(expected=expected, query=query), self.assertRaises(ValueError):
                    parse_reply(make_query(0, query), 0, expected)
            for actual, frame in replies.items():
                if actual != expected:
                    with (
                        self.subTest(expected=expected, actual=actual),
                        self.assertRaises(ValueError),
                    ):
                        parse_reply(frame, 0, expected)

    def test_wrong_ids_and_invalid_expected_ids_for_all_fields(self):
        replies = {
            "position": b"#001P1513!",
            "mode": b"#001MOD1!",
            "version": b"#001PV0.97!",
            "id": b"#001P!",
        }
        for field, frame in replies.items():
            with self.subTest(field=field), self.assertRaises(ValueError):
                parse_reply(frame, 0, field)
            for expected in (True, False, -1, 6, "1", 1.0, None):
                with self.subTest(field=field, expected=expected), self.assertRaises(ValueError):
                    parse_reply(frame, expected, field)
            for malformed_id in (b"#006", b"#255", b"#00x"):
                with self.subTest(field=field, prefix=malformed_id), self.assertRaises(ValueError):
                    parse_reply(malformed_id + frame[4:], 0, field)

    def test_malformed_mode_and_version_replies(self):
        invalid_modes = (
            b"#000MOD0!",
            b"#000MOD9!",
            b"#000MOD01!",
            b"#000PMOD0!",
            b"#000PMOD9!",
            b"#000MODx!",
            b"#000MOD1!\r\n",
            b"#000MOD1!!",
        )
        invalid_versions = (
            b"#000PV!",
            b"#000@ !",
            b"#000@   !",
            b"#000@ZL!",
            b"#000PV0 97!",
            b"#000PV0.97\r!",
            b"#000PV\xff!",
            b"#000@ ZL\x00!",
            b"#000PV0.97!!",
            b"#000@ ZL!\r\n",
        )
        for field, frames in (("mode", invalid_modes), ("version", invalid_versions)):
            for frame in frames:
                with self.subTest(field=field, frame=frame), self.assertRaises(ValueError):
                    parse_reply(frame, 0, field)
        for field in ("mode", "version", "id", "position"):
            for frame in (b"", b"#!", b"000P!", b"#000P", None, "#000P!"):
                with self.subTest(field=field, frame=frame), self.assertRaises(ValueError):
                    parse_reply(frame, 0, field)
        for field in ("stop", "reset", "", True, None, [], b"id"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                parse_reply(b"#000P!", 0, field)


class MotionCommandTests(unittest.TestCase):
    def test_historical_command_and_edges(self):
        self.assertEqual(make_move_command(0, 1555, 1500), b"#000P1555T1500!")
        self.assertEqual(make_move_command(0, 500, 20), b"#000P0500T0020!")
        self.assertEqual(make_move_command(5, 2500, 9999), b"#005P2500T9999!")
        for servo_id in range(6):
            for position_p in (500, 1500, 2500):
                for duration_ms in (20, 1500, 9999):
                    fields = (servo_id, position_p, duration_ms)
                    with self.subTest(fields=fields):
                        self.assertEqual(
                            validate_motion_command(make_move_command(*fields)), fields
                        )

    def test_all_numeric_fields_reject_bool_and_non_int(self):
        for index in range(3):
            for invalid in (True, False, 1.0, "1", None):
                fields = [0, 1500, 1500]
                fields[index] = invalid
                with self.subTest(fields=fields), self.assertRaises(ValueError):
                    make_move_command(*fields)

    def test_field_boundaries(self):
        invalid = (
            (-1, 1500, 1500),
            (6, 1500, 1500),
            (255, 1500, 1500),
            (0, 499, 1500),
            (0, 2501, 1500),
            (0, 1500, 0),
            (0, 1500, 19),
            (0, 1500, 10000),
        )
        for fields in invalid:
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                make_move_command(*fields)

    def test_validation_rejects_other_commands_and_unsupported_values(self):
        invalid = (
            b"#000PRAD!",
            b"#000PDST!",
            b"#000PULK!",
            b"#000PMOD1!",
            b"#000PID001!",
            b"#000PCLE!",
            b"$DST!",
            b"$DGS:0!",
            b"#255P1500T1500!",
            b"#006P1500T1500!",
            b"#000P0499T1500!",
            b"#000P2501T1500!",
            b"#000P1500T0000!",
            b"#000P1500T0019!",
            b"#000P1500T10000!",
            b"#000P+500T1500!",
            b"#00xP1500T1500!",
            b"#000P1500T15x0!",
            b"{#000P1500T1500!}",
            b"#000P1500T1500!\r\n",
            b"#000P1500T1500!#001P1500T1500!",
            b"",
            "#000P1500T1500!",
            bytearray(b"#000P1500T1500!"),
            None,
        )
        for command in invalid:
            with self.subTest(command=command), self.assertRaises(ValueError):
                validate_motion_command(command)


class FrameReceiverTests(unittest.TestCase):
    def test_historical_two_chunk_reply(self):
        receiver = FrameReceiver()
        # 2026-09-28 的两条 RX 日志；这是历史回放，非当前设备状态。
        self.assertEqual(receiver.feed(bytes.fromhex("23")), [])
        self.assertEqual(receiver.buffer, b"#")
        self.assertEqual(receiver.feed(bytes.fromhex("303030503135313321")), [b"#000P1513!"])
        self.assertEqual(receiver.buffer, b"")

    def test_all_two_piece_splits_and_byte_by_byte(self):
        frames = [b"#000P1513!", b"#005P1534!"]
        data = b"".join(frames)
        for split in range(len(data) + 1):
            receiver = FrameReceiver()
            with self.subTest(split=split):
                self.assertEqual(receiver.feed(data[:split]) + receiver.feed(data[split:]), frames)
        receiver = FrameReceiver()
        result = []
        for byte in data:
            result.extend(receiver.feed(bytes([byte])))
        self.assertEqual(result, frames)

    def test_noise_multiple_frames_and_saved_tail(self):
        receiver = FrameReceiver()
        frames = receiver.feed(b"noise\xff!\r\n#000P1513!#001P1499!\r\n#002P")
        self.assertEqual(frames, [b"#000P1513!", b"#001P1499!"])
        self.assertEqual(receiver.buffer, b"#002P")
        self.assertEqual(receiver.feed(b"1494!"), [b"#002P1494!"])

    def test_new_start_resynchronizes_incomplete_frame(self):
        receiver = FrameReceiver()
        self.assertEqual(receiver.feed(b"#000P15##001P0500!"), [b"#001P0500!"])
        self.assertEqual(receiver.buffer, b"")

    def test_exact_size_and_overlong_frames_with_recovery(self):
        exact = b"#" + b"a" * 62 + b"!"
        too_long = b"#" + b"a" * 63 + b"!"
        receiver = FrameReceiver()
        self.assertEqual(receiver.feed(exact), [exact])
        self.assertEqual(receiver.feed(too_long + b"#000P1513!"), [b"#000P1513!"])
        self.assertEqual(receiver.feed(b"#" + b"a" * 64), [])
        self.assertEqual(receiver.buffer, b"")
        self.assertEqual(receiver.feed(b"ignored!#005P1534!"), [b"#005P1534!"])

    def test_buffer_stays_bounded_for_unterminated_input(self):
        receiver = FrameReceiver(max_frame_bytes=10)
        for chunk in (b"#", b"a" * 7, b"a" * 3, b"a" * 1000):
            self.assertEqual(receiver.feed(chunk), [])
            self.assertLessEqual(len(receiver.buffer), 10)
        self.assertEqual(receiver.feed(b"#000P0500!"), [b"#000P0500!"])

    def test_framing_does_not_accept_business_semantics(self):
        frames = FrameReceiver().feed(b"#000PRAD!#001P1500!#000P9999!#000P1513!")
        self.assertEqual(len(frames), 4)
        for frame in frames[:-1]:
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                parse_position_reply(frame, 0)
        self.assertEqual(parse_position_reply(frames[-1], 0), 1513)

    def test_chunk_types_empty_chunk_and_invalid_limit(self):
        receiver = FrameReceiver()
        self.assertEqual(receiver.feed(bytearray(b"#000P")), [])
        self.assertEqual(receiver.feed(b""), [])
        self.assertEqual(receiver.buffer, b"#000P")
        self.assertEqual(receiver.feed(memoryview(b"1513!")), [b"#000P1513!"])
        for chunk in ("#000P1513!", [35, 33], 35, None):
            with self.subTest(chunk=chunk), self.assertRaises(ValueError):
                receiver.feed(chunk)
        for limit in (True, False, 0, 1, -1, 64.0, "64", None):
            with self.subTest(limit=limit), self.assertRaises(ValueError):
                FrameReceiver(limit)
        self.assertEqual(FrameReceiver(2).feed(b"#!"), [b"#!"])


if __name__ == "__main__":
    unittest.main()
