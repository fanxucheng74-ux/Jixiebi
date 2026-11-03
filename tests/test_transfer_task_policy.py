"""显式有界策略的独立复核；全部输入为模拟规划，不建立硬件观测或实物许可。"""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.general_transfer_entry import _duration, plan_simulated_general_entry
from arm_project.handoff_envelope import HandoffEnvelopeError
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_task_motion import TransferTaskSpec, validate_task_review
from arm_project.transfer_task_plan import compose_transfer_task
from arm_project.transfer_task_policy import (
    EVIDENCE_PREFIX,
    build_policy_review,
    canonical_sha256,
    cycle_motion_sha256,
)


ROOT = Path(__file__).resolve().parents[1]


def read_config(name):
    return json.loads((ROOT / "configs" / name).read_text(encoding="utf-8"))


class TransferTaskPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base_model = read_config("kinematics.arduino_nominal.json")
        cls.base_scene = read_config("motion_scene.example.json")
        cls.base_planner = read_config("general_entry_planner.example.json")
        cls.entry_policy = read_config("transfer_entry_policy.example.json")
        cls.transfer = plan_fixed_transfer(
            cls.base_model, read_config("transfer_task_request.example.json")
        )
        cls.base_task = cls.make_task()

    @classmethod
    def make_task(cls, start=None, via=None):
        positions = [1500] * 6 if start is None else list(start)
        entry = plan_simulated_general_entry(
            cls.base_model,
            cls.transfer,
            positions,
            cls.entry_policy,
            cls.base_scene,
            cls.base_planner,
        )
        if via is not None:
            # 明确重建完整模拟路由；策略必须独立检查全部路点，不能只信原规划成功标志。
            route = [positions[:4], list(via), entry["target_pick_hover_p"]]
            entry["route4_p"] = route
            entry["entry_segments"] = [
                {"from4_p": a, "to4_p": b, "duration_ms": _duration(a, b, cls.base_planner)}
                for a, b in zip(route, route[1:])
            ]
            entry["entry_declared_ms"] = sum(p["duration_ms"] for p in entry["entry_segments"])
            entry["entry_and_transfer_declared_ms"] = (
                entry["entry_declared_ms"] + cls.transfer["declared_ms_per_transfer"]
            )
        return compose_transfer_task(
            cls.transfer,
            entry,
            {"open_p": 1200, "close_p": 1570, "duration_ms": 1000},
            "POLICY_SIMULATION_ONLY",
        )

    def setUp(self):
        self.model, self.scene, self.planner = deepcopy(
            (self.base_model, self.base_scene, self.base_planner)
        )
        self.task = deepcopy(self.base_task)
        self.policy = {
            "schema": "jixiebi.transfer_task_policy.v1",
            "enabled": True,
            "evidence": "Explicit synthetic task policy for offline tests; no hardware observations.",
            "model_sha256": canonical_sha256(self.model),
            "scene_sha256": canonical_sha256(self.scene),
            "planner_sha256": canonical_sha256(self.planner),
            "cycle_motion_sha256": cycle_motion_sha256(self.task),
            "observation_bounds_p": [[500, 2500] for _ in range(6)],
            # 精确内存设备的3P身体走廊仅用于数学通过对照，不修改现场20/35P配置。
            "handoff_tolerance_p": {"0": 3, "1": 3, "2": 3, "3": 3, "5": 20},
            "maximum_entry_segments": 8,
            "deadline_ms": 100000,
        }

    def build(self):
        return build_policy_review(self.task, self.model, self.scene, self.planner, self.policy)

    def test_full_five_cycle_policy_review_validates_without_mutating_nominal_task(self):
        before = deepcopy((self.task, self.model, self.scene, self.planner, self.policy))
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            review = self.build()
        self.assertEqual((self.task, self.model, self.scene, self.planner, self.policy), before)
        self.assertEqual(review["schema"], "jixiebi.reviewed_transfer_task.v1")
        self.assertTrue(review["whole_path_reviewed_for_motion"])
        self.assertTrue(review["evidence"].startswith(EVIDENCE_PREFIX))
        evidence = json.loads(review["evidence"][len(EVIDENCE_PREFIX) :])
        self.assertGreater(evidence["sampled_scene_checks"], 1000)
        self.assertFalse(evidence["physical_observation_evidence"])
        self.assertFalse(evidence["physical_geometry_verified"])
        self.assertFalse(evidence["asynchronous_motion_envelope_verified"])
        self.assertFalse(self.task["motion_authorized"])
        self.assertEqual(self.task["source"], "SIMULATION")
        self.assertIsNone(self.task["entry_freshness"])
        self.assertEqual(review["initial_setpoints_p"], {})
        self.assertEqual(review["start_intervals_p"], [[1497, 1503] for _ in range(6)])
        spec = validate_task_review(self.task, review)
        self.assertEqual(TransferTaskSpec.from_dict(review), spec)
        self.assertEqual(len(spec.phases), 42)

    def test_policy_is_exact_and_explicitly_enabled_with_bounded_parameters(self):
        mutations = (
            ("enabled", False),
            ("enabled", 1),
            ("evidence", "  "),
            ("maximum_entry_segments", 9),
            ("maximum_entry_segments", True),
            ("deadline_ms", 180001),
            ("handoff_tolerance_p", {str(i): 20 for i in range(6)}),
            ("handoff_tolerance_p", {str(i): 2 for i in (0, 1, 2, 3, 5)}),
            ("observation_bounds_p", [[500, 2500]] * 5),
            ("model_sha256", "X" * 64),
            ("extra", True),
        )
        original = deepcopy(self.policy)
        for key, value in mutations:
            self.policy = {**original, key: value}
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.build()
        self.policy = {k: v for k, v in original.items() if k != "evidence"}
        with self.assertRaises(ValueError):
            self.build()

    def test_model_scene_planner_and_cycle_hashes_reject_every_change(self):
        for field in ("model", "scene", "planner", "cycle_target", "cycle_duration"):
            self.setUp()
            if field == "model":
                self.model["geometry_mm"]["tool_mm"] += 1
            elif field == "scene":
                self.scene["maximum_radial_mm"] += 1
            elif field == "planner":
                self.planner["resolution_p"] += 1
            elif field == "cycle_target":
                self.task["motion_phases"][-1]["targets"]["0"] += 1
            else:
                self.task["motion_phases"][-1]["duration_ms"] += 1
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.build()

    def test_cycle_hash_excludes_new_start_and_entry_route_but_binds_preparation(self):
        goal = self.transfer["nominal_endpoints_p"]["pick_hover"]
        already = self.make_task(goal + [1500, 1500])
        routed = self.make_task(via=[1500, *goal[1:]])
        original = cycle_motion_sha256(self.task)
        self.assertEqual(cycle_motion_sha256(already), original)
        self.assertEqual(cycle_motion_sha256(routed), original)
        self.assertEqual(canonical_sha256({"a": 1, "b": [2]}), canonical_sha256({"b": [2], "a": 1}))
        self.task["motion_phases"][1]["duration_ms"] += 1
        self.assertNotEqual(cycle_motion_sha256(self.task), original)
        with self.assertRaises(ValueError):
            canonical_sha256({"nan": float("nan")})

    def test_complete_routed_entry_compiles_every_segment_and_tracks_goals_only(self):
        goal = self.transfer["nominal_endpoints_p"]["pick_hover"]
        self.task = self.make_task(via=[1500, *goal[1:]])
        review = self.build()
        self.assertEqual(len(review["phases"]), 43)
        first, second = review["phases"][:2]
        self.assertEqual([first["name"], second["name"]], ["entry_01", "entry_02"])
        self.assertEqual(first["hold_tracking_p"], {})
        self.assertEqual(second["hold_tracking_p"], {"1": 3, "2": 3, "3": 3})
        self.assertEqual(
            {str(t["servo_id"]): t["target_p"] for t in second["targets"]},
            {str(i): p for i, p in enumerate(goal)},
        )
        self.assertEqual(review["initial_setpoints_p"], {})
        validate_task_review(self.task, review)

    def test_start_bounds_are_clipped_and_out_of_bounds_current_or_frames_fail(self):
        self.policy["observation_bounds_p"][4] = [1500, 1501]
        review = self.build()
        self.assertEqual(review["start_intervals_p"][4], [1500, 1501])
        self.policy["observation_bounds_p"][0] = [1501, 2500]
        with self.assertRaisesRegex(ValueError, "current task state"):
            self.build()
        self.policy["observation_bounds_p"][0] = [500, 2500]
        self.policy["observation_bounds_p"][1] = [1400, 2500]
        with self.assertRaisesRegex(ValueError, "phase state"):
            self.build()

    def test_changed_route_metadata_unbound_entry_timing_and_segment_cap_fail(self):
        original = deepcopy(self.task)
        self.task["entry_route4_p"][0][0] += 1
        with self.assertRaisesRegex(ValueError, "route differs"):
            self.build()
        self.task = deepcopy(original)
        self.task["phases"][0]["duration_ms"] = 1500
        self.task["motion_phases"][0]["duration_ms"] = 1500
        with self.assertRaisesRegex(ValueError, "planner dynamics"):
            self.build()
        goal = self.transfer["nominal_endpoints_p"]["pick_hover"]
        self.task = self.make_task(via=[1500, *goal[1:]])
        self.policy["maximum_entry_segments"] = 1
        with self.assertRaisesRegex(ValueError, "segment count"):
            self.build()

    def test_complete_state_motion_divergence_or_missing_general_marker_fail(self):
        original = deepcopy(self.task)
        for change in (
            "marker",
            "current",
            "full_target",
            "motion_target",
            "chain",
            "entry_prefix",
        ):
            self.task = deepcopy(original)
            if change == "marker":
                self.task.pop("entry_planner_version")
            elif change == "current":
                self.task["current_positions6_p"][4] += 1
            elif change == "full_target":
                self.task["phases"][0]["targets"]["5"] += 1
            elif change == "motion_target":
                self.task["phases"][0]["motion_targets"]["1"] += 1
            elif change == "chain":
                self.task["phases"][1]["start_targets"]["0"] += 1
            else:
                self.task["phases"][-1]["phase_id"] = "entry_hidden"
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.build()

    def test_invalid_bound_scene_and_rebound_wrong_model_cannot_supply_review(self):
        self.scene["link_radii_mm"][0] = 0
        self.policy["scene_sha256"] = canonical_sha256(self.scene)
        with self.assertRaises(ValueError):
            self.build()
        self.scene = deepcopy(self.base_scene)
        self.policy["scene_sha256"] = canonical_sha256(self.scene)
        self.model["geometry_mm"]["tool_mm"] += 1
        self.policy["model_sha256"] = canonical_sha256(self.model)
        with self.assertRaisesRegex(ValueError, "policy model"):
            self.build()

    def test_scene_revalidation_checks_entry_interior_even_when_endpoints_are_valid(self):
        self.scene["obstacles_aabb_mm"] = [
            {"min": [-7.36, 17.69, 203.18], "max": [-7.26, 17.79, 203.28]}
        ]
        validator = SceneValidator(self.model, self.scene)
        self.assertTrue(validator([1500] * 4))
        self.assertTrue(validator(self.transfer["nominal_endpoints_p"]["pick_hover"]))
        self.policy["scene_sha256"] = canonical_sha256(self.scene)
        with self.assertRaisesRegex(ValueError, "path collision"):
            self.build()

    def test_scene_revalidation_covers_fixed_transfer_after_valid_entry(self):
        # 低位取物点的盒体不阻塞高位入口；整趟策略仍须拒绝后续下降阶段。
        self.scene["obstacles_aabb_mm"] = [{"min": [-142, 140, 11], "max": [-140, 142, 14]}]
        validator = SceneValidator(self.model, self.scene)
        self.assertTrue(validator([1500] * 4))
        self.assertTrue(validator(self.transfer["nominal_endpoints_p"]["pick_hover"]))
        self.assertFalse(validator(self.transfer["nominal_endpoints_p"]["pick"]))
        self.policy["scene_sha256"] = canonical_sha256(self.scene)
        with self.assertRaisesRegex(ValueError, "path collision"):
            self.build()

    def test_already_at_start_acceptance_has_no_entry_motion_or_fabricated_goals(self):
        goal = self.transfer["nominal_endpoints_p"]["pick_hover"]
        self.task = self.make_task(goal + [1500, 1500])
        review = self.build()
        self.assertEqual(review["phases"][0]["name"], "prepare_gripper")
        self.assertEqual(review["initial_setpoints_p"], {})
        self.assertFalse(any(p["name"].startswith("entry") for p in review["phases"]))

    def test_zero_time_entry_cannot_hide_uncommanded_handoff_sized_displacement(self):
        goal = self.transfer["nominal_endpoints_p"]["pick_hover"]
        original = self.make_task(goal + [1500, 1500])
        self.policy["handoff_tolerance_p"] = {str(sid): 80 for sid in (0, 1, 2, 3, 5)}
        for offset in (4, 5, 80):
            self.task = deepcopy(original)
            self.task["current_positions6_p"][0] += offset
            self.task["phases"][0]["start_targets"]["0"] += offset
            self.task["entry_route4_p"][0][0] += offset
            with self.subTest(offset=offset), self.assertRaisesRegex(ValueError, "unsent axis"):
                self.build()
        self.task = deepcopy(original)
        self.task["current_positions6_p"][0] += 3
        self.task["phases"][0]["start_targets"]["0"] += 3
        self.task["entry_route4_p"][0][0] += 3
        # 3P以内的未发送起点差异合法，但80P交接盒仍不能用于低位搬运。
        with self.assertRaises(HandoffEnvelopeError):
            self.build()
        self.policy["handoff_tolerance_p"] = {"0": 3, "1": 3, "2": 3, "3": 3, "5": 20}
        self.assertEqual(self.build()["phases"][0]["name"], "prepare_gripper")

    def test_wide_fixture_tolerances_keep_their_values_and_expose_a_table_counterexample(self):
        # 合成容差夹具没有启用策略或设备授权，不依赖本机.site配置。
        site_tolerances = json.loads(
            (ROOT / "tests/fixtures/handoff_tolerances.json").read_text(encoding="utf-8")
        )["handoff_tolerance_p"]
        self.policy["handoff_tolerance_p"] = deepcopy(site_tolerances)
        before = deepcopy((self.task, self.model, self.scene, self.planner, self.policy))
        with self.assertRaises(HandoffEnvelopeError) as caught:
            self.build()
        self.assertEqual((self.task, self.model, self.scene, self.planner, self.policy), before)
        report = caught.exception.report
        self.assertEqual(report["status"], "COUNTEREXAMPLE")
        self.assertEqual(report["failure"]["phase"], "cycle_01_descend_left")
        # 检查器保存第一个违反5mm要求的反例，不要求它搜索最负的角点。
        self.assertLess(
            report["failure"]["counterexample"]["minimum_table_clearance_mm"],
            report["required_table_clearance_mm"],
        )
        self.assertEqual(site_tolerances, {"0": 20, "1": 20, "2": 35, "3": 35, "5": 40})

    def test_scene_check_count_and_monotonic_budgets_fail_closed(self):
        with (
            patch("arm_project.transfer_task_policy._MAX_VALIDITY_CHECKS", 1),
            self.assertRaisesRegex(ValueError, "scene-check budget"),
        ):
            self.build()
        with (
            patch("arm_project.transfer_task_policy.time.monotonic", side_effect=[0, 4]),
            self.assertRaisesRegex(ValueError, "monotonic"),
        ):
            self.build()


if __name__ == "__main__":
    unittest.main()
