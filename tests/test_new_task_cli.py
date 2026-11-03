"""主动新任务编号的CLI边界；不打开串口，也不改原配置或旧预算。"""

import contextlib
import io
import unittest
from unittest.mock import patch

from arm_project.cli.run_transfer_task import main


class NewTaskCliTests(unittest.TestCase):
    def test_explicit_new_task_changes_only_in_memory_id(self):
        original = {
            "task_id": "existing_used_task",
            "task_policy_path": "policy.json",
            "gripper": {"open_p": 1200},
        }
        completed = {
            "status": "RESPONSE_SEQUENCE_COMPLETED",
            "response_pass": True,
            "result_path": "fixture.json",
            "error": None,
        }
        with (
            patch("arm_project.cli.run_transfer_task.read_json", return_value=original),
            patch("arm_project.cli.run_transfer_task.TransferTaskRunner") as runner,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            runner.return_value.run.return_value = completed
            self.assertEqual(main(["--execute", "--new-task", "--port", "NEVER_OPENED"]), 0)
        passed = runner.call_args.args[0]
        self.assertRegex(passed["task_id"], r"^pose_check_[a-f0-9]{32}$")
        self.assertEqual(original["task_id"], "existing_used_task")
        self.assertEqual(
            {k: v for k, v in passed.items() if k != "task_id"},
            {k: v for k, v in original.items() if k != "task_id"},
        )
        runner.return_value.run.assert_called_once()

    def test_static_review_or_nonexecute_cannot_create_new_motion_attempt(self):
        with (
            patch(
                "arm_project.cli.run_transfer_task.read_json", return_value={"task_id": "static"}
            ),
            patch("arm_project.cli.run_transfer_task.TransferTaskRunner") as runner,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(main(["--execute", "--new-task", "--port", "NEVER_OPENED"]), 1)
            runner.assert_not_called()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            main(["--new-task", "--demo", "upright"])
        self.assertEqual(error.exception.code, 2)
