"""真实名义模型的低余量入口与整趟任务模拟；不连接任何实物设备。"""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.clearance_recovery import TableRecovery
from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.general_transfer_entry import plan_simulated_general_entry
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_task_motion import validate_task_review
from arm_project.transfer_task_plan import compose_transfer_task
from arm_project.transfer_task_policy import (
    EVIDENCE_PREFIX,
    build_policy_review,
    canonical_sha256,
    cycle_motion_sha256,
)
from arm_project.transfer_task_runtime import TransferTaskRunner, demo_task, read_json, save_new
import test_transfer_task_runtime as runtime_fixtures


ROOT = Path(__file__).resolve().parents[1]
SOFT_START = [1833, 1331, 2163, 1190]


def config(name):
    return read_json(ROOT / "configs" / name)


def task_policy(model, scene, planner, task):
    return {
        "schema": "jixiebi.transfer_task_policy.v1",
        "enabled": True,
        "evidence": "SIMULATION recovery fixture; no measured clearance or physical authorization",
        "model_sha256": canonical_sha256(model),
        "scene_sha256": canonical_sha256(scene),
        "planner_sha256": canonical_sha256(planner),
        "cycle_motion_sha256": cycle_motion_sha256(task),
        "observation_bounds_p": [[500, 2500] for _ in range(6)],
        # 仅精确模拟器使用3P身体容差，不代表实物具有3P精度，也不修改现场配置。
        "handoff_tolerance_p": {"0": 3, "1": 3, "2": 3, "3": 3, "5": 20},
        "maximum_entry_segments": 8,
        "deadline_ms": 100000,
    }


class TableRecoveryPlanningIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.model = config("kinematics.arduino_nominal.json")
        self.scene_config = config("motion_scene.example.json")
        self.planner = config("general_entry_planner.example.json")
        self.entry_policy = config("transfer_entry_policy.example.json")
        self.transfer = plan_fixed_transfer(
            self.model, config("transfer_task_request.example.json")
        )
        self.scene = SceneValidator(self.model, self.scene_config)

    def plan(self, positions=None):
        return plan_simulated_general_entry(
            self.model,
            self.transfer,
            (SOFT_START if positions is None else positions) + [1500, 1200],
            self.entry_policy,
            self.scene_config,
            self.planner,
        )

    def compose(self, entry):
        return compose_transfer_task(
            self.transfer,
            entry,
            {"open_p": 1200, "close_p": 1570, "duration_ms": 1000},
            "SIM-table-recovery",
        )

    def test_positive_soft_start_recovers_in_same_entry_and_preserves_composed_route(self):
        before = deepcopy((self.model, self.scene_config, self.planner, self.transfer))
        self.assertEqual(self.scene.table_z_mm, 0)
        start = self.scene.inspect(SOFT_START)
        self.assertEqual(start["code"], "TABLE_CLEARANCE")
        self.assertAlmostEqual(start["minimum_table_clearance_mm"], 2.2852059695368325)
        self.assertTrue(self.scene.inspect(SOFT_START, minimum_clearance_mm=0.5)["valid"])
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            entry = self.plan()
            task = self.compose(entry)
        self.assertEqual(entry["status"], "NOMINAL_ENTRY_PROPOSAL", entry["reason"])
        self.assertTrue(entry["table_recovery"]["eligible"])
        self.assertEqual(entry["table_recovery"]["hard_clearance_mm"], 0.5)
        self.assertEqual(entry["table_recovery"]["minimum_clearance_mm"], 5)
        self.assertEqual(entry["table_recovery"]["escape_search"]["status"], "ESCAPE_FOUND")
        self.assertTrue(entry["table_recovery"]["route_check"]["normal_margin_reached"])
        self.assertTrue(
            TableRecovery(self.scene, SOFT_START, resolution_p=1).validate_route(entry["route4_p"])[
                "valid"
            ]
        )
        self.assertEqual(entry["route4_p"][0], SOFT_START)
        self.assertEqual(entry["route4_p"][-1], self.transfer["nominal_endpoints_p"]["pick_hover"])
        self.assertEqual(task["entry_route4_p"], entry["route4_p"])
        entries = [p for p in task["motion_phases"] if p["name"].startswith("entry_")]
        self.assertEqual(len(entries), len(entry["route4_p"]) - 1)
        self.assertEqual(
            [[phase["targets"][str(i)] for i in range(4)] for phase in entries],
            entry["route4_p"][1:],
        )
        self.assertEqual(task["entry_diagnostics"]["table_recovery"], entry["table_recovery"])
        for phase in task["phases"]:
            if not phase["phase_id"].startswith("entry"):
                self.assertTrue(self.scene([phase["targets"][str(i)] for i in range(4)]))
        self.assertEqual(self.scene_config["minimum_clearance_mm"], 5)
        self.assertEqual((self.model, self.scene_config, self.planner, self.transfer), before)
        self.assertFalse(task["motion_authorized"])
        self.assertFalse(task["physical_path_verified"])

    def test_subhard_negative_and_hidden_self_collision_starts_produce_no_motion_route(self):
        for start, expected_hard_code in (
            ([1833, 1331, 2163, 1180], "TABLE_CLEARANCE"),
            ([1833, 1331, 2163, 1171], "TABLE_CLEARANCE"),
            ([1500, 1842, 2451, 834], "SELF_COLLISION"),
        ):
            with self.subTest(start=start):
                self.assertEqual(self.scene.inspect(start)["code"], "TABLE_CLEARANCE")
                self.assertEqual(
                    self.scene.inspect(start, minimum_clearance_mm=0.5)["code"], expected_hard_code
                )
                entry = self.plan(start)
                self.assertEqual(entry["status"], "NOMINAL_PATH_BLOCKED")
                self.assertEqual(entry["route4_p"], [])
                self.assertEqual(entry["entry_segments"], [])
                self.assertFalse(entry["table_recovery"]["eligible"])
                self.assertEqual(self.compose(entry)["phases"], [])

    def test_obstacle_at_soft_start_cannot_be_hidden_by_ordinary_table_margin_failure(self):
        tip = self.scene.inspect(SOFT_START)["chain_points_mm"][-1]
        self.scene_config["obstacles_aabb_mm"] = [
            {"min": [v - 0.1 for v in tip], "max": [v + 0.1 for v in tip]}
        ]
        scene = SceneValidator(self.model, self.scene_config)
        self.assertEqual(scene.inspect(SOFT_START)["code"], "TABLE_CLEARANCE")
        self.assertEqual(
            scene.inspect(SOFT_START, minimum_clearance_mm=0.5)["code"], "OBSTACLE_COLLISION"
        )
        self.assertTrue(scene(self.transfer["nominal_endpoints_p"]["pick_hover"]))
        entry = self.plan()
        self.assertEqual(entry["status"], "NOMINAL_PATH_BLOCKED")
        self.assertFalse(entry["table_recovery"]["eligible"])
        self.assertEqual(entry["entry_segments"], [])

    def test_falsely_successful_escape_with_intermediate_dip_is_revalidated_and_blocked(self):
        endpoint = [1833, 1028, 2114, 1779]
        self.assertTrue(self.scene(endpoint))
        self.assertLess(
            self.scene.inspect([1833, 1328, 2163, 1195])["minimum_table_clearance_mm"],
            self.scene.inspect(SOFT_START)["minimum_table_clearance_mm"],
        )
        goal = self.transfer["nominal_endpoints_p"]["pick_hover"]
        with (
            patch(
                "arm_project.general_transfer_entry.plan_table_escape",
                return_value={"status": "ESCAPE_FOUND", "waypoints4_p": [SOFT_START, endpoint]},
            ),
            patch(
                "arm_project.general_transfer_entry.plan_joint_path",
                return_value={
                    "status": "PATH_FOUND",
                    "waypoints4_p": [endpoint, goal],
                    "stats": {"method": "SIMULATED_INVALID_ESCAPE"},
                },
            ),
        ):
            entry = self.plan()
        self.assertEqual(entry["status"], "NOMINAL_PATH_BLOCKED")
        self.assertEqual(entry["route4_p"], [])
        self.assertEqual(entry["entry_segments"], [])
        self.assertEqual(entry["samples"], [])

    def test_bound_policy_independently_reviews_escape_and_ordinary_five_cycle_path(self):
        task = self.compose(self.plan())
        policy = task_policy(self.model, self.scene_config, self.planner, task)
        review = build_policy_review(task, self.model, self.scene_config, self.planner, policy)
        validate_task_review(task, review)
        evidence = json.loads(review["evidence"][len(EVIDENCE_PREFIX) :])
        self.assertFalse(evidence["physical_observation_evidence"])
        self.assertGreater(evidence["sampled_scene_checks"], 1000)
        self.assertEqual(self.scene.minimum_clearance_mm, 5)
        self.assertFalse(task["physical_cycles_verified"])


class TableRecoveryRuntimeIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.fixture = runtime_fixtures.TransferTaskRuntimeTests("runTest")
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.setUp()
        self.config = self.fixture.config
        self.scene = config("motion_scene.example.json")
        self.planner = config("general_entry_planner.example.json")
        for key, name, value in (
            ("scene_path", "scene.json", self.scene),
            ("entry_planner_path", "planner.json", self.planner),
        ):
            path = self.fixture.folder / name
            save_new(path, value)
            self.config[key] = str(path)
        baseline = demo_task(self.config, "upright")
        policy = task_policy(self.fixture.model, self.scene, self.planner, baseline)
        policy_path = self.fixture.folder / "task_policy.json"
        save_new(policy_path, policy)
        self.config["task_policy_path"] = str(policy_path)
        self.fixture.raw.initial = SOFT_START + [1500, 1200]

    def run_task(self, progress=None):
        with self.fixture.runtime_environment():
            return TransferTaskRunner(self.config, "SIM-RECOVERY-PORT", progress=progress).run()

    def test_one_fake_serial_owner_recovers_then_completes_five_cycles_and_records_exact_budget(
        self,
    ):
        result = self.run_task()
        self.assertEqual(result["status"], "RESPONSE_SEQUENCE_COMPLETED", result["error"])
        self.assertTrue(result["response_pass"])
        self.assertTrue(result["transport_closed"])
        self.assertEqual((self.fixture.raw.open_count, self.fixture.raw.close_count), (1, 1))
        self.assertEqual(len(self.fixture.links), 1)
        self.assertEqual(len(self.fixture.gates), 1)
        self.assertIs(self.fixture.gates[0].raw, self.fixture.raw)
        task = read_json(self.fixture.run_folder / "task_plan.json")
        self.assertEqual(task["current_positions6_p"][:4], SOFT_START)
        self.assertTrue(task["entry_diagnostics"]["table_recovery"]["eligible"])
        self.assertEqual(result["sequence"]["phases"][0]["name"], "entry_01")
        self.assertEqual(result["sequence"]["phases"][-1]["name"], "cycle_05_rotate_left")
        self.assertEqual(len(result["cycle_elapsed_s"]), 5)
        budget = read_json(self.fixture.run_folder / "motion_attempt.json")
        frames = [frame.encode() for phase in budget["plan"]["commands"] for frame in phase]
        self.assertEqual(self.fixture.motions(self.fixture.raw), frames)
        self.assertFalse(result["physical_task_verified"])
        self.assertFalse(result["physical_stop_verified"])
        self.assertFalse(task["physical_cycles_verified"])
        self.assertEqual(read_json(self.config["scene_path"])["minimum_clearance_mm"], 5)

    def test_observed_return_to_soft_margin_after_escape_stops_before_cycles(self):
        failed = False
        original_position = self.fixture.raw.position

        def progress(value):
            nonlocal failed
            if value.get("phase") == "prepare_gripper":
                failed = True

        def observed_position(sid):
            return SOFT_START[sid] if failed and sid < 4 else original_position(sid)

        with patch.object(self.fixture.raw, "position", side_effect=observed_position):
            result = self.run_task(progress=progress)
        self.assertTrue(failed)
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT", result["error"])
        self.assertEqual(result["cycle_elapsed_s"], [])
        self.assertFalse(any(p["name"].startswith("cycle_") for p in result["sequence"]["phases"]))
        self.assertTrue(result["transport_closed"])
        self.assertTrue((self.fixture.run_folder / "motion_attempt.json").is_file())
        self.assertFalse(result["physical_stop_verified"])


if __name__ == "__main__":
    unittest.main()
