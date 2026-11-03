"""Read-only retrieval checks use small text fixtures and no project imports."""

import hashlib
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools import project_context
from tools.project_context import build_context


def put(root, relative, text):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


class ProjectContextTests(unittest.TestCase):
    def test_protocol_retrieval_preserves_numbered_lines_hashes_and_scope(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            put(root, "docs/PROJECT_ASSISTANT_SCOPE.md", "当前只读问答，不连接机械臂。\n")
            path = put(
                root,
                "src/arm_project/servo_protocol.py",
                'READ = b"PRAD"\n# 位置查询 #004PRAD!\ndef make_move():\n'
                '    return b"#004P1540T1500!"\n# P不是实测角度，T是毫秒\n',
            )
            result = build_context(root, "PRAD查询与P/T运动帧有什么区别？")
            self.assertEqual(result["sources"][0]["path"], "docs/PROJECT_ASSISTANT_SCOPE.md")
            source = result["sources"][1]
            self.assertEqual(source["path"], "src/arm_project/servo_protocol.py")
            self.assertEqual(source["sha256"], hashlib.sha256(path.read_bytes()).hexdigest())
            original = path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(
                source["text"].splitlines(),
                original[source["start_line"] - 1 : source["end_line"]],
            )
            self.assertIn("[S2] src/arm_project/servo_protocol.py:L1-L5", result["context_text"])
            self.assertIn('4:     return b"#004P1540T1500!"', result["context_text"])

    def test_limits_include_headers_and_no_selected_chunks_overlap(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            put(
                root,
                "src/arm_project/servo_protocol.py",
                "\n".join(f"# PRAD 位置查询 {number}" for number in range(150)),
            )
            result = build_context(root, "PRAD", max_chars=750, max_sources=2)
            self.assertLessEqual(len(result["context_text"]), 750)
            self.assertLessEqual(len(result["sources"]), 2)
            for index, first in enumerate(result["sources"]):
                for second in result["sources"][index + 1 :]:
                    self.assertTrue(first["end_line"] < second["start_line"])

    def test_only_allowlisted_sources_are_read_and_latest_content_is_used(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = put(root, "src/arm_project/servo_protocol.py", "# PRAD current 1\n")
            for relative in (
                ".env",
                "docs/OLD_PROTOCOL.md",
                "data/datasets/train.messages.jsonl",
                "firmware/wifi_config.py",
                "runs/raw.jsonl",
                "src/arm_project/wifi_config.py",
            ):
                put(root, relative, "SECRET PRAD current 999\n")
            first = build_context(root, "PRAD")
            self.assertNotIn("SECRET", first["context_text"])
            self.assertEqual(first["scanned_files"], ["src/arm_project/servo_protocol.py"])
            source.write_text("# PRAD current 2\n", encoding="utf-8")
            second = build_context(root, "PRAD")
            self.assertNotEqual(first["sources"][0]["sha256"], second["sources"][0]["sha256"])
            self.assertIn("current 2", second["context_text"])

    def test_rejects_linked_source_before_reading_target(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = put(root, "src/arm_project/servo_protocol.py", "# PRAD\n")
            original = project_context._is_link
            with patch(
                "tools.project_context._is_link",
                side_effect=lambda path: os.path.samefile(path, source) or original(path),
            ):
                with self.assertRaisesRegex(ValueError, "linked project source"):
                    build_context(root, "PRAD")

    def test_actual_linked_source_is_rejected_before_reading(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = put(root, "src/arm_project/servo_protocol.py", "# original\n")
            outside = root / "outside.py"
            outside.write_text("# PRAD secret\n", encoding="utf-8")
            source.unlink()
            try:
                source.symlink_to(outside)
            except OSError as error:
                self.skipTest(f"creating an actual symlink is unavailable: {error}")
            with self.assertRaisesRegex(ValueError, "linked project source"):
                build_context(root, "PRAD")

    def test_scope_is_never_shortened_to_fit_a_small_budget(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scope = "\n".join("重要约束必须完整保留" + str(index) for index in range(25))
            put(root, "docs/PROJECT_ASSISTANT_SCOPE.md", scope)
            put(root, "src/arm_project/servo_protocol.py", "# PRAD position\n")
            with self.assertRaisesRegex(ValueError, "mandatory scope"):
                build_context(root, "PRAD", max_chars=256)
            result = build_context(root, "PRAD", max_chars=1000)
            self.assertEqual(result["sources"][0]["text"], scope)

    def test_invalid_arguments_and_no_evidence_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for kwargs in (
                {"question": ""},
                {"question": " "},
                {"question": True},
                {"question": "x" * 4001},
                {"question": "PRAD", "topic": "other"},
                {"question": "PRAD", "max_chars": 0},
                {"question": "PRAD", "max_chars": 24001},
                {"question": "PRAD", "max_chars": True},
                {"question": "PRAD", "max_sources": 0},
                {"question": "PRAD", "max_sources": 13},
            ):
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    build_context(root, **kwargs)
            with self.assertRaisesRegex(ValueError, "no relevant"):
                build_context(root, "PRAD")

    def test_large_file_and_oversized_single_line_never_bypass_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = put(root, "src/arm_project/servo_protocol.py", "x" * (256 * 1024 + 1))
            with self.assertRaisesRegex(ValueError, "byte limit"):
                build_context(root, "PRAD")
            source.write_text("X" * 3000 + "\n# PRAD whole line\n", encoding="utf-8")
            result = build_context(root, "PRAD")
            self.assertEqual(result["sources"][0]["start_line"], 2)
            self.assertEqual(result["sources"][0]["text"], "# PRAD whole line")
            self.assertNotIn("XXX", result["context_text"])

    def test_auto_selects_tcp_and_missing_optional_sources_are_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            put(root, "src/arm_project/protocol.py", "# TCP JSON frame command_id\n")
            result = build_context(root, "TCP socket如何分帧？", topic="auto")
            self.assertEqual(result["effective_topic"], "tcp")
            self.assertEqual(result["sources"][0]["path"], "src/arm_project/protocol.py")

    def test_literal_move_frame_and_reply_intents_keep_complete_protocol_functions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            put(root, "docs/PROJECT_ASSISTANT_SCOPE.md", "只读项目问答。\n")
            put(
                root,
                "docs/ARCHITECTURE.md",
                "\n".join("查询 回包 P/T 运动帧 单位和限制 " * 5 for _ in range(40)),
            )
            put(
                root,
                "src/arm_project/servo_protocol.py",
                "# Initial protocol introduction\n"
                + "\n" * 50
                + "def parse_position_reply(frame, expected_id):\n"
                + "    # 回包格式验证\n    return expected_id\n"
                + "\n" * 50
                + "def make_move_command(servo_id, position_p, duration_ms):\n"
                + "    require_range(duration_ms, 20, 9999)\n"
                + "    return servo_id, position_p, duration_ms\n",
            )
            result = build_context(
                root,
                "解释 #004PRAD! 与 #004P1540T1500!，P/T单位和限制，回包是否证明到位？",
                max_sources=4,
            )
            code = [row for row in result["sources"] if row["path"].endswith("servo_protocol.py")]
            self.assertEqual(len(code), 2)
            self.assertEqual(
                {row.get("symbol") for row in code}, {"make_move_command", "parse_position_reply"}
            )
            self.assertTrue(
                any("require_range(duration_ms, 20, 9999)" in row["text"] for row in code)
            )


if __name__ == "__main__":
    unittest.main()
