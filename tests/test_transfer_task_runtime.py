"""整趟任务入口的离线集成验收；伪设备的 HARDWARE 标签仅测试来源门禁。

所有收发由内存 Device 和 FakeClock 提供；测试没有打开真实串口、相机或证明
实物取放。临时 home/预算目录均隔离在仓库 artifacts 下并在测试结束后清理。
"""

from contextlib import contextmanager, ExitStack
from copy import deepcopy
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from arm_project.arm_link import ServoLink
from arm_project.cli.run_transfer_task import main
from arm_project.servo_protocol import validate_read_query
from arm_project.transfer_task_motion import TaskTransport
from arm_project.transfer_task_runtime import (
    TransferTaskRunner,
    demo_task,
    load_settings,
    read_json,
    review_draft,
    save_new,
)
from test_arm_driver import FakeClock
from test_timed_cycle import Device


ROOT = Path(__file__).resolve().parents[1]
BOOT_ID = "e48131fb-887b-4981-bf22-28ca73985563"


class RuntimeDevice(Device):
    # 只为通过真实入口的来源校验；字节仍完全由 Device 内存模拟，不能作为实物证据。
    source = "HARDWARE"

    def __init__(self, clock):
        super().__init__(clock)
        self.open_count = self.close_count = 0

    def open(self):
        self.open_count += 1
        if not self.closed:
            raise RuntimeError("simulated raw serial must never be opened twice")
        super().open()

    def close(self):
        self.close_count += 1
        super().close()


class TransferTaskRuntimeTests(unittest.TestCase):
    def setUp(self):
        # 公开检出不携带artifacts；本测试显式建立临时模拟输出的父目录。
        (ROOT / "artifacts").mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(prefix="runtime_test_", dir=ROOT / "artifacts")
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.home = self.folder / "fake_home"
        self.project = self.home / "Jixiebi"
        self.project.mkdir(parents=True)
        (self.project / ".arm-project-root").write_text("simulation only\n", encoding="utf-8")
        self.config = read_json(ROOT / "configs/transfer_task.example.json")
        self.config["task_id"] = "SIM-runtime"
        for key in ("model_path", "request_path", "policy_path"):
            self.config[key] = str(ROOT / self.config[key])
        self.clock = FakeClock()
        self.raw = RuntimeDevice(self.clock)
        self.model, self.transfer, _ = load_settings(self.config)
        self.set_posture("right_hover")
        self.gates = []
        self.links = []

    @property
    def run_folder(self):
        return self.project / "runs/transfer_tasks" / self.config["task_id"]

    def set_posture(self, posture):
        if posture == "upright":
            positions = [1500] * 4
        else:
            key = "place_hover" if posture == "right_hover" else "pick_hover"
            positions = self.transfer["nominal_endpoints_p"][key]
        self.raw.initial = list(positions) + [1500, 1200]

    @staticmethod
    def motions(raw):
        return [command for command in raw.writes if len(command) == 15 and b"T" in command]

    def make_review(self, posture):
        task = demo_task(self.config, posture)
        review = review_draft(task, self.model)
        key = "place_hover" if posture == "right_hover" else "pick_hover"
        current = self.transfer["nominal_endpoints_p"][key] + [1500, 1200]
        review.update(
            whole_path_reviewed_for_motion=True,
            evidence="SIMULATION fixture only; not a physical authorization or acceptance",
            start_intervals_p=[[position - 2, position + 2] for position in current],
            observation_bounds_p=[[500, 2500] for _ in range(6)],
            deadline_ms=70000,
        )
        for phase in review["phases"]:
            for target in phase["targets"]:
                target["end_tolerance_p"] = 20
        return review

    def install_reviews(self):
        for status, posture in (
            ("ALREADY_AT_START", "left_hover"),
            ("NOMINAL_ENTRY_PROPOSAL", "right_hover"),
        ):
            path = self.folder / f"{status}.json"
            save_new(path, self.make_review(posture))
            self.config["reviews"][status] = str(path)

    @contextmanager
    def runtime_environment(self):
        original_read_text = Path.read_text

        def fake_read_text(path, *args, **kwargs):
            if str(path).replace("\\", "/") == "/proc/sys/kernel/random/boot_id":
                return BOOT_ID
            return original_read_text(path, *args, **kwargs)

        def fake_link(transport, record=None):
            link = ServoLink(transport, record=record, clock=self.clock)
            self.links.append(link)
            return link

        def exact_gate(port, spec, *, raw):
            gate = TaskTransport(port, spec, raw=raw)
            self.gates.append(gate)
            return gate

        with ExitStack() as stack:
            stack.enter_context(patch.object(Path, "home", return_value=self.home))
            stack.enter_context(patch.object(Path, "read_text", new=fake_read_text))
            raw_factory = stack.enter_context(
                patch("arm_project.transfer_task_runtime._TaskSerial", return_value=self.raw)
            )
            stack.enter_context(
                patch("arm_project.transfer_task_runtime.ServoLink", side_effect=fake_link)
            )
            stack.enter_context(
                patch("arm_project.transfer_task_runtime.TaskTransport", side_effect=exact_gate)
            )
            yield raw_factory

    def test_construct_and_demo_do_not_open_serial_or_consume_motion_budget(self):
        before = deepcopy(self.config)
        with patch("arm_project.transfer_task_runtime._TaskSerial") as raw_factory:
            runner = TransferTaskRunner(self.config, "NEVER_OPENED")
            task = demo_task(self.config, "right_hover")
            raw_factory.assert_not_called()
        self.assertFalse(runner.used)
        self.assertEqual(task["source"], "SIMULATION")
        self.assertFalse(task["motion_authorized"])
        self.assertEqual(task["cycle_count"], 5)
        self.assertEqual(self.config, before)
        self.assertFalse(self.run_folder.exists())

    def test_cli_demo_writes_nominal_task_without_transport_and_missing_mode_is_rejected(self):
        config_path = self.folder / "config.json"
        save_new(config_path, self.config)
        output = self.folder / "demo"
        with patch("arm_project.transfer_task_runtime._TaskSerial") as raw_factory:
            with patch("sys.stdout", new_callable=io.StringIO):
                code = main(
                    [
                        "--config",
                        str(config_path),
                        "--demo",
                        "right_hover",
                        "--output-dir",
                        str(output),
                    ]
                )
            self.assertEqual(code, 0)
            raw_factory.assert_not_called()
        task = read_json(output / "task_plan.json")
        self.assertEqual(task["source"], "SIMULATION")
        self.assertFalse(task["hardware_execution_allowed"])
        with patch("sys.stderr", new_callable=io.StringIO), self.assertRaises(SystemExit):
            main(["--config", str(config_path)])

    def test_missing_review_is_selected_after_fresh_state_and_sends_only_queries(self):
        updates = []
        runner = TransferTaskRunner(self.config, "SIM-PORT", progress=updates.append)
        with self.runtime_environment() as raw_factory:
            result = runner.run()
        self.assertEqual(result["status"], "REVIEW_REQUIRED", result["error"])
        self.assertFalse(result["response_pass"])
        self.assertEqual(self.motions(self.raw), [])
        self.assertEqual(len(self.raw.writes), 6)
        for command in self.raw.writes:
            validate_read_query(command)
        raw_factory.assert_called_once_with("SIM-PORT")
        self.assertEqual((self.raw.open_count, self.raw.close_count), (1, 1))
        self.assertTrue(result["transport_closed"])
        draft = read_json(self.run_folder / "review_draft.json")
        self.assertFalse(draft["whole_path_reviewed_for_motion"])
        self.assertIsNone(draft["start_intervals_p"])
        self.assertFalse((self.run_folder / "motion_attempt.json").exists())
        self.assertEqual(updates[0]["status"], "READING_CURRENT_STATE")
        self.assertEqual(updates[-1]["status"], "REVIEW_REQUIRED")

    def test_left_branch_ignores_right_review_and_reuses_same_open_raw_under_exact_gate(self):
        self.install_reviews()
        self.set_posture("left_hover")
        right_path = Path(self.config["reviews"]["NOMINAL_ENTRY_PROPOSAL"])
        right_path.write_text("not valid JSON; this branch must not be read", encoding="utf-8")
        runner = TransferTaskRunner(self.config, "SIM-PORT")
        with self.runtime_environment():
            result = runner.run()
        self.assertTrue(result["response_pass"], result["error"])
        self.assertEqual(len(self.links), 1)
        self.assertEqual(len(self.gates), 1)
        self.assertIs(self.links[0].transport, self.gates[0])
        self.assertIs(self.gates[0].raw, self.raw)
        self.assertEqual((self.raw.open_count, self.raw.close_count), (1, 1))
        self.assertEqual(len(result["sequence"]["phases"]), 41)
        self.assertEqual(result["sequence"]["phases"][0]["name"], "prepare_gripper")
        self.assertTrue((self.run_folder / "motion_attempt.json").is_file())

    def test_right_entry_and_five_cycles_complete_once_with_unverified_physical_flags(self):
        self.install_reviews()
        runner = TransferTaskRunner(self.config, "SIM-PORT")
        with self.runtime_environment():
            result = runner.run()
        self.assertEqual(result["status"], "RESPONSE_SEQUENCE_COMPLETED", result["error"])
        self.assertTrue(result["response_pass"])
        self.assertTrue(result["transport_closed"])
        self.assertFalse(result["physical_task_verified"])
        self.assertFalse(result["physical_stop_verified"])
        sequence = result["sequence"]
        self.assertEqual(len(sequence["phases"]), 42)
        self.assertEqual(sequence["phases"][0]["name"], "entry")
        self.assertEqual(sequence["phases"][-1]["name"], "cycle_05_rotate_left")
        self.assertEqual(len(result["cycle_elapsed_s"]), 5)
        self.assertTrue(
            all(cycle["drive_sequence_completed"] for cycle in result["cycle_elapsed_s"])
        )
        self.assertTrue(all(cycle["elapsed_s"] > 0 for cycle in result["cycle_elapsed_s"]))
        self.assertTrue(
            all(not cycle["physical_grasp_verified"] for cycle in result["cycle_elapsed_s"])
        )
        budget = read_json(self.run_folder / "motion_attempt.json")
        expected = [command.encode() for phase in budget["plan"]["commands"] for command in phase]
        self.assertEqual(self.motions(self.raw), expected)
        self.assertEqual(
            [row["position_p"] for row in sequence["final_state"]["servos"][:4]],
            self.transfer["nominal_endpoints_p"]["pick_hover"],
        )

    def test_existing_task_directory_does_not_gain_or_overwrite_result_budget_or_logs(self):
        self.run_folder.mkdir(parents=True)
        for name in ("config.json", "motion_attempt.json", "events.jsonl"):
            (self.run_folder / name).write_bytes(b"preserve failed attempt exactly\n")
        before = {path.name: path.read_bytes() for path in self.run_folder.iterdir()}
        with self.runtime_environment() as raw_factory:
            result = TransferTaskRunner(self.config, "SIM-PORT").run()
        self.assertEqual(result["status"], "FAILED")
        self.assertIsNone(result.get("result_path"))
        self.assertEqual(
            {path.name: path.read_bytes() for path in self.run_folder.iterdir()}, before
        )
        raw_factory.assert_not_called()

    def test_same_runner_and_new_runner_cannot_replay_or_replace_completed_task(self):
        self.install_reviews()
        runner = TransferTaskRunner(self.config, "SIM-PORT")
        with self.runtime_environment() as raw_factory:
            first = runner.run()
            before = {path.name: path.read_bytes() for path in self.run_folder.iterdir()}
            same = runner.run()
            again = TransferTaskRunner(self.config, "SIM-PORT").run()
            raw_factory.assert_called_once()
        self.assertTrue(first["response_pass"], first["error"])
        self.assertEqual(same["status"], "ALREADY_USED")
        self.assertFalse(again["response_pass"])
        self.assertEqual(
            {path.name: path.read_bytes() for path in self.run_folder.iterdir()}, before
        )
        self.assertEqual(self.raw.open_count, 1)

    def test_active_runner_rejects_reentrant_start_without_a_second_serial_owner(self):
        self.install_reviews()
        nested = []
        runner = None

        def progress(value):
            if value["status"] == "READING_CURRENT_STATE":
                nested.append(runner.run())

        runner = TransferTaskRunner(self.config, "SIM-PORT", progress=progress)
        with self.runtime_environment():
            result = runner.run()
        self.assertTrue(result["response_pass"], result["error"])
        self.assertEqual(nested, [{"status": "ALREADY_USED", "response_pass": False}])
        self.assertEqual(self.raw.open_count, 1)

    def test_nonhardware_current_state_never_reaches_motion_gate(self):
        self.install_reviews()
        self.raw.source = "SIMULATION"
        with self.runtime_environment():
            result = TransferTaskRunner(self.config, "SIM-PORT").run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["status"], "FAILED")
        self.assertEqual(self.motions(self.raw), [])
        self.assertEqual(self.gates, [])
        self.assertTrue(self.raw.closed)
        self.assertFalse((self.run_folder / "motion_attempt.json").exists())

    def test_invalid_initial_reply_leaves_incomplete_state_and_blocks_before_motion_gate(self):
        self.install_reviews()
        self.raw.initial[2] = 499
        with self.runtime_environment():
            result = TransferTaskRunner(self.config, "SIM-PORT").run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["status"], "FAILED")
        self.assertIn("incomplete", result["error"])
        self.assertEqual(self.motions(self.raw), [])
        self.assertEqual(self.gates, [])
        self.assertTrue(self.raw.closed)

    def test_unknown_upright_entry_is_blocked_before_review_or_motion(self):
        self.install_reviews()
        self.set_posture("upright")
        with self.runtime_environment():
            result = TransferTaskRunner(self.config, "SIM-PORT").run()
        self.assertEqual(result["status"], "ENTRY_BLOCKED", result["error"])
        self.assertEqual(self.motions(self.raw), [])
        self.assertEqual(self.gates, [])
        self.assertTrue(self.raw.closed)

    def test_wrong_branch_review_never_creates_gate_or_sends_motion(self):
        self.install_reviews()
        self.config["reviews"]["NOMINAL_ENTRY_PROPOSAL"] = self.config["reviews"][
            "ALREADY_AT_START"
        ]
        with self.runtime_environment():
            result = TransferTaskRunner(self.config, "SIM-PORT").run()
        self.assertFalse(result["response_pass"])
        self.assertIn("review does not match", result["error"])
        self.assertEqual(self.motions(self.raw), [])
        self.assertEqual(self.gates, [])
        self.assertTrue(self.raw.closed)
        self.assertFalse((self.run_folder / "motion_attempt.json").exists())

    def test_cancel_before_run_never_opens_transport_or_consumes_task_directory(self):
        runner = TransferTaskRunner(self.config, "SIM-PORT")
        self.assertEqual(runner.cancel()["status"], "CANCEL_REQUESTED")
        with self.runtime_environment() as raw_factory:
            result = runner.run()
            raw_factory.assert_not_called()
        self.assertEqual(result["status"], "CANCELLED")
        self.assertFalse(self.run_folder.exists())

    def test_cancel_after_open_before_initial_query_closes_without_queries_or_motion(self):
        runner = None

        def progress(value):
            if value["status"] == "READING_CURRENT_STATE":
                runner.cancel()

        runner = TransferTaskRunner(self.config, "SIM-PORT", progress=progress)
        with self.runtime_environment():
            result = runner.run()
        self.assertEqual(result["status"], "CANCELLED")
        self.assertEqual(self.raw.writes, [])
        self.assertEqual((self.raw.open_count, self.raw.close_count), (1, 1))
        self.assertEqual(runner.cancel()["status"], "FINISHED")

    def test_cancel_during_cycle_stops_attempted_axes_once_and_never_runs_next_phase(self):
        self.install_reviews()
        runner = None

        def progress(value):
            if value["status"] == "RUNNING" and value["phase"] == "cycle_01_descend_left":
                runner.cancel()

        runner = TransferTaskRunner(self.config, "SIM-PORT", progress=progress)
        with self.runtime_environment():
            result = runner.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["sequence"]["phases"][-1]["name"], "cycle_01_descend_left")
        self.assertEqual(result["cycle_elapsed_s"], [])
        self.assertEqual(len(self.motions(self.raw)), 8)
        for sid in (0, 1, 2, 3, 5):
            self.assertEqual(self.raw.writes.count(f"#{sid:03d}PDST!".encode()), 1)
        self.assertEqual(self.raw.writes.count(b"#004PDST!"), 0)
        self.assertTrue(self.raw.closed)

    def test_persistent_budget_failure_prevents_all_motion(self):
        self.install_reviews()

        def fail_budget(path, value):
            if path.name == "motion_attempt.json":
                # 模拟预算落盘中断；已创建的尝试记录必须保留且不能用于自动重发。
                path.write_text("partial persisted attempt", encoding="utf-8")
                raise OSError("simulated fsync budget failure")
            return save_new(path, value)

        with self.runtime_environment():
            with patch("arm_project.transfer_task_runtime.save_new", side_effect=fail_budget):
                result = TransferTaskRunner(self.config, "SIM-PORT").run()
        self.assertFalse(result["response_pass"])
        self.assertIn("budget failure", result["error"])
        self.assertEqual(self.motions(self.raw), [])
        self.assertFalse(self.gates[0].armed)
        self.assertTrue(self.raw.closed)
        self.assertEqual(
            (self.run_folder / "motion_attempt.json").read_text(encoding="utf-8"),
            "partial persisted attempt",
        )

    def test_progress_callback_failure_before_state_query_is_recorded_and_closes_transport(self):
        def progress(value):
            if value["status"] == "READING_CURRENT_STATE":
                raise OSError("simulated initial progress delivery failure")

        runner = TransferTaskRunner(self.config, "SIM-PORT", progress=progress)
        with self.runtime_environment():
            result = runner.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(self.raw.writes, [])
        self.assertTrue(result["transport_closed"])
        self.assertEqual(result["logging_errors"], [])
        self.assertEqual(
            result["progress_errors"], ["OSError: simulated initial progress delivery failure"]
        )
        self.assertEqual(
            read_json(self.run_folder / "result.json")["progress_errors"],
            result["progress_errors"],
        )

    def test_progress_callback_failure_during_entry_stops_once_and_blocks_later_phases(self):
        self.install_reviews()

        def progress(value):
            if value["status"] == "RUNNING" and value["phase"] == "entry":
                raise OSError("simulated active progress delivery failure")

        runner = TransferTaskRunner(self.config, "SIM-PORT", progress=progress)
        with self.runtime_environment():
            result = runner.run()
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["sequence"]["phases"][-1]["name"], "entry")
        self.assertEqual(len(self.motions(self.raw)), 4)
        self.assertEqual(
            result["progress_errors"], ["OSError: simulated active progress delivery failure"]
        )
        for sid in range(4):
            self.assertEqual(self.raw.writes.count(f"#{sid:03d}PDST!".encode()), 1)
        self.assertEqual(self.raw.writes.count(b"#005PDST!"), 0)
        self.assertTrue(result["transport_closed"])
        self.assertEqual(
            read_json(self.run_folder / "result.json")["progress_errors"],
            result["progress_errors"],
        )

    def test_final_progress_callback_failure_is_reflected_in_return_and_persisted_result(self):
        self.install_reviews()

        def progress(value):
            if value["status"] == "RESPONSE_SEQUENCE_COMPLETED":
                raise OSError("simulated final progress delivery failure")

        runner = TransferTaskRunner(self.config, "SIM-PORT", progress=progress)
        with self.runtime_environment():
            result = runner.run()
        self.assertTrue(result["sequence"]["response_pass"])
        self.assertFalse(result["response_pass"])
        self.assertNotEqual(result["status"], "RESPONSE_SEQUENCE_COMPLETED")
        self.assertEqual(
            result["progress_errors"], ["OSError: simulated final progress delivery failure"]
        )
        self.assertTrue(result["transport_closed"])
        stored = read_json(self.run_folder / "result.json")
        self.assertEqual(stored["progress_errors"], result["progress_errors"])
        self.assertEqual(stored["status"], result["status"])
        self.assertFalse(stored["response_pass"])

    def test_invalid_review_records_cleanup_metadata_in_failure_result(self):
        self.install_reviews()
        review_path = Path(self.config["reviews"]["NOMINAL_ENTRY_PROPOSAL"])
        review_path.write_text("{ invalid review JSON", encoding="utf-8")
        with self.runtime_environment():
            result = TransferTaskRunner(self.config, "SIM-PORT").run()
        self.assertEqual(result["status"], "FAILED")
        self.assertFalse(result["response_pass"])
        self.assertTrue(result["transport_closed"])
        self.assertEqual(result["logging_errors"], [])
        self.assertEqual(result["progress_errors"], [])
        self.assertEqual(self.motions(self.raw), [])
        stored = read_json(self.run_folder / "result.json")
        self.assertTrue(stored["transport_closed"])
        self.assertEqual(stored["logging_errors"], [])

    def test_nonzero_table_request_is_rejected_before_serial_or_budget_creation(self):
        request = read_json(self.config["request_path"])
        request["table_z_mm"] = 25
        request_path = self.folder / "unsupported_table.json"
        save_new(request_path, request)
        self.config["request_path"] = str(request_path)
        with self.assertRaisesRegex(ValueError, "table_z_mm|桌面"):
            load_settings(self.config)
        with self.runtime_environment() as raw_factory:
            result = TransferTaskRunner(self.config, "SIM-PORT").run()
            raw_factory.assert_not_called()
        self.assertEqual(result["status"], "FAILED")
        self.assertFalse(result["response_pass"])
        self.assertFalse(self.run_folder.exists())


if __name__ == "__main__":
    unittest.main()
