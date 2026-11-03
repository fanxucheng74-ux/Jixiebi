"""阻塞任务也固化当前位置与场景证据；全部IO来自内存设备和临时目录。"""

from copy import deepcopy
import json
from pathlib import Path
import unittest

from arm_project.motion_scene import SceneValidator
from arm_project.servo_protocol import validate_read_query
from arm_project.transfer_task_runtime import TransferTaskRunner, read_json, save_new
import test_transfer_task_runtime as runtime_fixtures


ROOT = Path(__file__).resolve().parents[1]
PENETRATING_START6_P = [1500, 834, 1800, 1500, 1500, 1200]


class RecoveryEvidenceTests(unittest.TestCase):
    def setUp(self):
        # 组合原fixture的FakeClock/内存Device/临时home，避免继承并重复其全部测试。
        self.fixture = runtime_fixtures.TransferTaskRuntimeTests("runTest")
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.setUp()
        self.config = self.fixture.config
        self.scene = read_json(ROOT / "configs/motion_scene.example.json")
        self.planner = read_json(ROOT / "configs/general_entry_planner.example.json")
        self.scene_path = self.fixture.folder / "scene.json"
        self.planner_path = self.fixture.folder / "planner.json"
        save_new(self.scene_path, self.scene)
        save_new(self.planner_path, self.planner)
        self.config.update(
            scene_path=str(self.scene_path), entry_planner_path=str(self.planner_path)
        )
        self.fixture.raw.initial = list(PENETRATING_START6_P)

    def test_blocked_entry_preserves_feedback_parameters_and_diagnostics_after_source_changes(self):
        expected_scene = deepcopy(self.scene)
        expected_start = SceneValidator(self.fixture.model, expected_scene).inspect(
            PENETRATING_START6_P[:4]
        )
        self.assertEqual(expected_start["code"], "TABLE_CLEARANCE")
        self.assertLess(expected_start["minimum_table_clearance_mm"], 0)
        runner = TransferTaskRunner(self.config, "SIM-EVIDENCE-PORT")
        with self.fixture.runtime_environment() as raw_factory:
            result = runner.run()

        self.assertEqual(result["status"], "ENTRY_BLOCKED", result["error"])
        self.assertFalse(result["response_pass"])
        self.assertTrue(result["transport_closed"])
        self.assertEqual(result["logging_errors"], [])
        raw_factory.assert_called_once_with("SIM-EVIDENCE-PORT")
        self.assertEqual((self.fixture.raw.open_count, self.fixture.raw.close_count), (1, 1))
        self.assertEqual(self.fixture.gates, [])
        self.assertIsNone(runner.controller)
        self.assertEqual(self.fixture.raw.moves, {})
        self.assertEqual(self.fixture.motions(self.fixture.raw), [])
        self.assertEqual(len(self.fixture.raw.writes), 6)
        for command in self.fixture.raw.writes:
            validate_read_query(command)

        folder = self.fixture.run_folder
        self.assertEqual(read_json(folder / "config.json"), self.config)
        entry = read_json(folder / "entry_plan.json")
        snapshot = read_json(folder / "input_snapshot.json")
        task = read_json(folder / "task_plan.json")
        self.assertEqual(entry["current_positions6_p"], PENETRATING_START6_P)
        self.assertEqual(entry["start_scene_check"], expected_start)
        self.assertEqual(len(entry["start_scene_check"]["links"]), 3)
        failed_link = entry["start_scene_check"]["link_index"]
        self.assertLess(
            entry["start_scene_check"]["links"][failed_link]["minimum_table_clearance_mm"],
            expected_scene["minimum_clearance_mm"],
        )
        self.assertFalse(entry["table_recovery"]["eligible"])
        self.assertFalse(entry["table_recovery"]["hard_scene_check"]["valid"])
        self.assertEqual(snapshot["model"], self.fixture.model)
        self.assertEqual(snapshot["request"], self.fixture.transfer["input_assumptions"])
        self.assertEqual(snapshot["entry_policy"], read_json(Path(self.config["policy_path"])))
        self.assertEqual(snapshot["scene_config"], expected_scene)
        self.assertEqual(snapshot["settings"], self.planner)
        self.assertEqual(task["status"], "BLOCKED")
        self.assertEqual(task["current_positions6_p"], PENETRATING_START6_P)
        self.assertEqual(task["entry_diagnostics"]["start_scene_check"], expected_start)
        self.assertEqual(task["entry_diagnostics"]["table_recovery"], entry["table_recovery"])
        self.assertEqual(task["entry_diagnostics"]["goal_scene_check"], entry["goal_scene_check"])
        self.assertEqual(task["motion_phases"], [])
        self.assertEqual(read_json(folder / "result.json")["status"], "ENTRY_BLOCKED")
        for name in ("motion_attempt.json", "review.json", "review_draft.json"):
            self.assertFalse((folder / name).exists(), name)

        # 原始收发仍可独立恢复该次六轴反馈；阻塞不丢日志，也不触发定位或停止帧。
        events = [
            json.loads(line)
            for line in (folder / "events.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual(len([item for item in events if item["event"] == "tx"]), 6)
        self.assertTrue(any(item["event"] == "rx" for item in events))
        states = [item["state"] for item in events if item["event"] == "arm_state"]
        self.assertEqual(len(states), 1)
        self.assertEqual(
            [servo["position_p"] for servo in states[0]["servos"]], PENETRATING_START6_P
        )

        # 后续编辑同一路径的场景文件，不得反向改变失败时实际采用的参数和诊断。
        evidence_names = ("entry_plan.json", "input_snapshot.json", "task_plan.json")
        evidence_before = {name: (folder / name).read_bytes() for name in evidence_names}
        changed = deepcopy(expected_scene)
        changed.update(table_z_mm=123.4, minimum_clearance_mm=88)
        self.scene_path.write_text(json.dumps(changed), encoding="utf-8")
        self.assertEqual(read_json(self.scene_path), changed)
        self.assertEqual(
            {name: (folder / name).read_bytes() for name in evidence_names}, evidence_before
        )
        self.assertEqual(read_json(folder / "input_snapshot.json")["scene_config"], expected_scene)
        self.assertEqual(read_json(folder / "entry_plan.json")["start_scene_check"], expected_start)
        self.assertEqual(
            read_json(folder / "task_plan.json")["entry_diagnostics"]["start_scene_check"],
            expected_start,
        )


if __name__ == "__main__":
    unittest.main()
