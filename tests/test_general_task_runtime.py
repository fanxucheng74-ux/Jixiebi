"""启用哈希绑定策略后的整趟运行模拟；HARDWARE标签仅来自内存测试替身。

复用原运行测试的临时home、FakeClock和伪串口；未访问实际硬件，不把算法策略
生成的审阅记录、五轮驱动响应或名义场景通过写成实物抓取成功。
"""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.transfer_task_policy import canonical_sha256, cycle_motion_sha256
from arm_project.transfer_task_runtime import (
    TransferTaskRunner,
    demo_task,
    read_json,
    review_draft,
    save_new,
)
import test_transfer_task_runtime as runtime_fixtures


ROOT = Path(__file__).resolve().parents[1]


class GeneralTaskRuntimeTests(unittest.TestCase):
    def setUp(self):
        # 组合现有fixture，避免继承其全部测试而重复运行旧验收。
        self.fixture = runtime_fixtures.TransferTaskRuntimeTests("runTest")
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.setUp()
        self.config = self.fixture.config
        self.scene = read_json(ROOT / "configs/motion_scene.example.json")
        self.planner = read_json(ROOT / "configs/general_entry_planner.example.json")
        scene_path = self.fixture.folder / "scene.json"
        planner_path = self.fixture.folder / "planner.json"
        self.policy_path = self.fixture.folder / "task_policy.json"
        save_new(scene_path, self.scene)
        save_new(planner_path, self.planner)
        self.config.update(
            scene_path=str(scene_path),
            entry_planner_path=str(planner_path),
            task_policy_path=str(self.policy_path),
        )
        self.fixture.set_posture("upright")
        task = demo_task(self.config, "upright")
        self.policy = {
            "schema": "jixiebi.transfer_task_policy.v1",
            "enabled": True,
            "evidence": "SIMULATION fixture policy; no physical authorization or calibration",
            "model_sha256": canonical_sha256(self.fixture.model),
            "scene_sha256": canonical_sha256(self.scene),
            "planner_sha256": canonical_sha256(self.planner),
            "cycle_motion_sha256": cycle_motion_sha256(task),
            "observation_bounds_p": [[500, 2500] for _ in range(6)],
            # 仅精确内存设备采用3P作为数学通过对照，不把生产容差偷偷改窄。
            "handoff_tolerance_p": {"0": 3, "1": 3, "2": 3, "3": 3, "5": 20},
            "maximum_entry_segments": 8,
            "deadline_ms": 100000,
        }
        save_new(self.policy_path, self.policy)

    def run_task(self):
        runner = TransferTaskRunner(self.config, "SIM-GENERAL-PORT")
        with self.fixture.runtime_environment():
            result = runner.run()
        return result

    def assert_complete(self, result):
        self.assertEqual(result["status"], "RESPONSE_SEQUENCE_COMPLETED", result["error"])
        self.assertTrue(result["response_pass"])
        self.assertTrue(result["transport_closed"])
        self.assertEqual(result["logging_errors"], [])
        self.assertEqual((self.fixture.raw.open_count, self.fixture.raw.close_count), (1, 1))
        self.assertEqual(len(self.fixture.links), 1)
        self.assertEqual(len(self.fixture.gates), 1)
        self.assertIs(self.fixture.gates[0].raw, self.fixture.raw)
        self.assertIs(self.fixture.links[0].transport, self.fixture.gates[0])
        self.assertEqual(
            self.config["reviews"], {"ALREADY_AT_START": None, "NOMINAL_ENTRY_PROPOSAL": None}
        )
        task = read_json(self.fixture.run_folder / "task_plan.json")
        self.assertEqual(task["entry_planner_version"], "GENERAL_JOINT_SPACE_V2")
        self.assertEqual(task["current_positions6_p"][:4], [1500] * 4)
        entry_phases = [
            phase for phase in task["motion_phases"] if phase["name"].startswith("entry_")
        ]
        self.assertEqual(len(entry_phases), len(task["entry_route4_p"]) - 1)
        self.assertEqual(result["sequence"]["phases"][-1]["name"], "cycle_05_rotate_left")
        self.assertEqual(len(result["sequence"]["phases"]), len(entry_phases) + 41)
        self.assertEqual(
            [cycle["cycle_index"] for cycle in result["cycle_elapsed_s"]], [1, 2, 3, 4, 5]
        )
        self.assertTrue(
            all(cycle["drive_sequence_completed"] for cycle in result["cycle_elapsed_s"])
        )
        self.assertTrue(all(cycle["elapsed_s"] > 0 for cycle in result["cycle_elapsed_s"]))
        self.assertTrue(
            all(not cycle["physical_grasp_verified"] for cycle in result["cycle_elapsed_s"])
        )
        for key in ("physical_task_verified", "physical_stop_verified"):
            self.assertFalse(result[key])
        self.assertFalse(result["sequence"]["physical_task_verified"])
        self.assertFalse(result["sequence"]["strict_position_verified"])
        review = read_json(self.fixture.run_folder / "review.json")
        self.assertTrue(review["whole_path_reviewed_for_motion"])
        self.assertIn("BOUNDED_POLICY_VALIDATION_ONLY", review["evidence"])
        self.assertIn('"physical_observation_evidence": false', review["evidence"])
        budget = read_json(self.fixture.run_folder / "motion_attempt.json")
        frames = [frame.encode() for phase in budget["plan"]["commands"] for frame in phase]
        self.assertEqual(self.fixture.motions(self.fixture.raw), frames)
        self.assertEqual(len(budget["plan"]["commands"]), len(task["motion_phases"]))
        self.assertFalse((self.fixture.run_folder / "review_draft.json").exists())
        handoff = read_json(self.fixture.run_folder / "handoff_envelope.json")
        self.assertTrue(handoff["valid"])
        self.assertFalse(handoff["asynchronous_motion_envelope_verified"])
        return task

    def test_factory_upright_plans_and_runs_five_cycles_using_one_enabled_policy(self):
        result = self.run_task()
        task = self.assert_complete(result)
        self.assertFalse(task["motion_authorized"])
        self.assertFalse(task["hardware_execution_allowed"])
        self.assertFalse(task["gripper_contents_known"])
        self.assertEqual(task["entry_route4_p"][0], [1500] * 4)
        self.assertEqual(
            task["entry_route4_p"][-1], self.fixture.transfer["nominal_endpoints_p"]["pick_hover"]
        )

    def test_routed_entry_is_not_collapsed_before_dynamic_review_budget_or_execution(self):
        goal = self.fixture.transfer["nominal_endpoints_p"]["pick_hover"]
        route = [[1500] * 4, [1500, 1400, 1500, 1500], [goal[0], 1400, 1500, 1500], list(goal)]

        def routed_search(start, target, bounds, validator, **kwargs):
            self.assertEqual(list(start), route[0])
            self.assertEqual(list(target), route[-1])
            self.assertTrue(all(validator(point) for point in route))
            return {
                "status": "PATH_FOUND",
                "waypoints4_p": deepcopy(route),
                "continuous_collision_check": False,
                "motion_authorized": False,
                "stats": {"method": "RRT_CONNECT", "reason": "test selected finite route"},
            }

        # 搜索算法另有实际RRT测试；这里固定合法多段结果，专查运行接线是否保留全部路点。
        with patch("arm_project.general_transfer_entry.plan_joint_path", side_effect=routed_search):
            result = self.run_task()
        task = self.assert_complete(result)
        self.assertEqual(task["entry_route4_p"], route)
        self.assertEqual(
            [p["name"] for p in result["sequence"]["phases"][:4]],
            ["entry_01", "entry_02", "entry_03", "prepare_gripper"],
        )

    def test_disabled_policy_keeps_current_state_read_only_and_creates_unapproved_draft(self):
        self.policy["enabled"] = False
        self.policy_path.write_text(json.dumps(self.policy), encoding="utf-8")
        result = self.run_task()
        self.assertEqual(result["status"], "REVIEW_REQUIRED", result["error"])
        self.assertFalse(result["response_pass"])
        self.assertEqual(self.fixture.motions(self.fixture.raw), [])
        self.assertEqual(len(self.fixture.raw.writes), 6)
        self.assertEqual(self.fixture.gates, [])
        self.assertFalse((self.fixture.run_folder / "motion_attempt.json").exists())
        draft = read_json(self.fixture.run_folder / "review_draft.json")
        self.assertFalse(draft["whole_path_reviewed_for_motion"])
        self.assertTrue(result["transport_closed"])

    def test_scene_change_fails_bound_policy_before_motion_or_budget_reservation(self):
        changed = deepcopy(self.scene)
        changed["minimum_clearance_mm"] = 6
        Path(self.config["scene_path"]).write_text(json.dumps(changed), encoding="utf-8")
        result = self.run_task()
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["status"], "FAILED")
        self.assertIn("scene_sha256", result["error"])
        self.assertEqual(self.fixture.motions(self.fixture.raw), [])
        self.assertEqual(self.fixture.gates, [])
        self.assertFalse((self.fixture.run_folder / "motion_attempt.json").exists())
        self.assertTrue(result["transport_closed"])

    def test_feedback_scene_failure_stops_entry_before_gripper_or_cycle_motion(self):
        checks = []

        def execution_scene(positions):
            checks.append(positions)
            return not bool(self.fixture.raw.moves)

        with patch(
            # 注入执行期检查器；SceneValidator本身仍保留inspect供起点/恢复复核。
            "arm_project.transfer_task_runtime.task_scene_validator",
            return_value=execution_scene,
        ):
            result = self.run_task()
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
        self.assertIn("configured nominal scene", result["error"])
        self.assertEqual(result["sequence"]["phases"][-1]["name"], "entry_01")
        self.assertEqual(result["cycle_elapsed_s"], [])
        self.assertEqual(len(self.fixture.motions(self.fixture.raw)), 4)
        for sid in range(4):
            self.assertEqual(self.fixture.raw.writes.count(f"#{sid:03d}PDST!".encode()), 1)
        self.assertEqual(self.fixture.raw.writes.count(b"#005PDST!"), 0)
        self.assertTrue((self.fixture.run_folder / "motion_attempt.json").is_file())
        self.assertGreaterEqual(len(checks), 3)
        self.assertTrue(result["transport_closed"])
        self.assertFalse(result["physical_stop_verified"])

    def assert_handoff_blocked_before_motion(self, result):
        self.assertEqual(result["status"], "HANDOFF_BLOCKED", result["error"])
        self.assertFalse(result["response_pass"])
        self.assertTrue(result["transport_closed"])
        self.assertEqual(self.fixture.motions(self.fixture.raw), [])
        self.assertEqual(len(self.fixture.raw.writes), 6)
        self.assertEqual(self.fixture.gates, [])
        self.assertFalse((self.fixture.run_folder / "motion_attempt.json").exists())
        self.assertFalse((self.fixture.run_folder / "review.json").exists())
        report = read_json(self.fixture.run_folder / "handoff_envelope.json")
        self.assertFalse(report["valid"])
        self.assertEqual(report["failure"]["phase"], "cycle_01_descend_left")
        self.assertEqual(report["status"], "COUNTEREXAMPLE")
        self.assertLess(
            report["failure"]["counterexample"]["minimum_table_clearance_mm"],
            report["required_table_clearance_mm"],
        )
        self.assertFalse(report["physical_collision_verified"])
        return report

    def test_wide_fixture_feedback_corridor_is_blocked_before_gate_or_motion_budget(self):
        # 只读取非敏感容差夹具；现场启用策略不随公开仓库分发。
        original_site = (ROOT / "tests/fixtures/handoff_tolerances.json").read_bytes()
        tolerances = json.loads(original_site)["handoff_tolerance_p"]
        self.policy["handoff_tolerance_p"] = deepcopy(tolerances)
        self.policy_path.write_text(json.dumps(self.policy), encoding="utf-8")
        result = self.run_task()
        self.assert_handoff_blocked_before_motion(result)
        self.assertEqual(read_json(self.policy_path)["handoff_tolerance_p"], tolerances)
        self.assertEqual(
            (ROOT / "tests/fixtures/handoff_tolerances.json").read_bytes(), original_site
        )

    def test_static_general_review_cannot_bypass_the_same_handoff_preflight(self):
        # 直接构造外部静态审阅，不经过新的动态策略；静态入口同样必须检查误差盒。
        task = demo_task(self.config, "upright")
        static_review = review_draft(task, self.fixture.model)
        tolerances = read_json(ROOT / "tests/fixtures/handoff_tolerances.json")[
            "handoff_tolerance_p"
        ]
        static_review.update(
            whole_path_reviewed_for_motion=True,
            evidence="SIMULATION external review fixture only; not physical authorization",
            start_intervals_p=[[p - 3, p + 3] for p in task["current_positions6_p"]],
            observation_bounds_p=[[500, 2500] for _ in range(6)],
            deadline_ms=100000,
        )
        for row in static_review["phases"]:
            for target in row["targets"]:
                target["end_tolerance_p"] = tolerances[str(target["servo_id"])]
        static_path = self.fixture.folder / "static_general_review.json"
        save_new(static_path, static_review)
        self.config.pop("task_policy_path")
        self.config["reviews"]["NOMINAL_ENTRY_PROPOSAL"] = str(static_path)
        result = self.run_task()
        self.assert_handoff_blocked_before_motion(result)


if __name__ == "__main__":
    unittest.main()
