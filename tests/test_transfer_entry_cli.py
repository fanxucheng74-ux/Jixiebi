"""入口命令演示验证；不连接串口或相机。"""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from arm_project.cli.plan_transfer_entry import main


class TransferEntryCliTests(unittest.TestCase):
    def test_right_demo_plans_entry_without_constructing_serial_session(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "result"
            with (
                patch(
                    "arm_project.ros_readonly.ReadOnlySession",
                    side_effect=AssertionError("demo cannot open hardware"),
                ),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--demo", "right_hover", "--output-dir", str(output)]), 0)
            result = json.loads((output / "entry_plan.json").read_text(encoding="utf-8"))
            self.assertFalse(result["hardware_motion"])
            self.assertFalse(result["entry"]["motion_authorized"])
            self.assertIsNone(result["raw_read_directory"])

    def test_left_demo_does_not_need_entry_and_output_is_not_reused(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "result"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--demo", "left_hover", "--output-dir", str(output)]), 0)
            before = (output / "entry_plan.json").read_bytes()
            result = json.loads(before)
            self.assertEqual(result["entry"]["status"], "ALREADY_AT_START")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--demo", "right_hover", "--output-dir", str(output)]), 1)
            self.assertEqual((output / "entry_plan.json").read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
