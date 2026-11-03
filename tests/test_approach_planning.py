"""Pure nominal planning and CLI fixtures; never use actual camera or serial I/O."""

from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
import itertools
import json
import math
from pathlib import Path
import random
import shutil
import unittest
from unittest.mock import patch
from uuid import uuid4

from arm_project.approach_planning import (
    chain_points,
    nominal_height_bounds,
    plan_approach,
    transition,
)
from arm_project.cli.preview_approach import main
from arm_project.kinematics import Geometry, forward, from_nominal_pulses
from arm_project.unified_arm import PROFILES

ROOT = Path(__file__).resolve().parents[1]


class ApproachTests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads((ROOT / "configs/kinematics.vendor_nominal.json").read_text())
        self.spec = json.loads((ROOT / "configs/approach.example.json").read_text())
        self.geometry = Geometry(**self.model["geometry_mm"])
        self.start = [1507, 1506, 1495, 1495]
        self.folder = ROOT / ("_approach_tests_" + uuid4().hex)
        self.folder.mkdir()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        path = self.folder.resolve()
        if path.parent != ROOT.resolve() or not path.name.startswith("_approach_tests_"):
            raise AssertionError("cleanup escaped owned directory")
        shutil.rmtree(path)

    def test_chain_lengths_and_tip_match_forward_on_multiple_poses(self):
        rng = random.Random(81)
        for _ in range(30):
            p = [rng.randint(1000, 2000) for _ in range(4)]
            points = chain_points(self.geometry, p)
            for a, b, length in zip(points, points[1:], (105, 75, 185)):
                self.assertAlmostEqual(math.dist(a, b), length, places=8)
            tip = forward(self.geometry, from_nominal_pulses(p))
            self.assertLess(math.dist(points[-1], [tip.x_mm, tip.y_mm, tip.z_mm]), 1e-8)

    def test_continuous_height_bound_contains_corners_and_upright_interior(self):
        windows = [PROFILES[i][:2] for i in range(4)]
        bounds = nominal_height_bounds(self.geometry, windows)
        for p in itertools.product(*windows):
            z = forward(self.geometry, from_nominal_pulses(p)).z_mm
            self.assertLessEqual(bounds["lower_mm"] - 1e-9, z)
            self.assertGreaterEqual(bounds["upper_mm"] + 1e-9, z)
        self.assertAlmostEqual(bounds["upper_mm"], 535)
        self.assertGreater(bounds["lower_mm"], 529)
        self.assertFalse(bounds["physical_height_verified"])

    def test_serial_transition_changes_one_axis_and_respects_step_bound(self):
        target = [1507, 1480, 1520, 1480]
        result = transition(
            self.geometry,
            self.model["nominal_angular_limits_deg"],
            self.start,
            target,
            max_step_p=10,
        )
        previous = self.start
        for step in result["steps"]:
            ending = step["end_nominal_p"]
            changed = [i for i in range(4) if ending[i] != previous[i]]
            self.assertEqual(changed, [step["servo_id"]])
            self.assertLessEqual(abs(ending[changed[0]] - previous[changed[0]]), 10)
            previous = ending
        self.assertEqual(previous, target)
        self.assertTrue(any(not s["within_current_command_window"] for s in result["steps"]))
        self.assertFalse(result["continuous_collision_check"])

    def test_same_pose_has_no_steps_but_still_checks_plane_and_angles(self):
        good = transition(
            self.geometry, self.model["nominal_angular_limits_deg"], self.start, self.start
        )
        self.assertEqual(good["steps"], [])
        with self.assertRaisesRegex(ValueError, "plane"):
            transition(
                self.geometry,
                self.model["nominal_angular_limits_deg"],
                self.start,
                self.start,
                floor_z_mm=200,
            )
        with self.assertRaisesRegex(ValueError, "angular"):
            transition(
                self.geometry,
                self.model["nominal_angular_limits_deg"],
                [1500, 700, 1500, 1500],
                self.start,
            )

    def test_invalid_pulses_step_and_plane_are_rejected(self):
        cases = [
            {"max_step_p": True},
            {"max_step_p": 0},
            {"max_step_p": 41},
            {"floor_z_mm": float("nan")},
            {"margin_mm": -1},
        ]
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                transition(
                    self.geometry,
                    self.model["nominal_angular_limits_deg"],
                    self.start,
                    self.start,
                    **changes,
                )
        with self.assertRaises(ValueError):
            nominal_height_bounds(self.geometry, [(1600, 1500)] * 4)

    def test_example_stages_are_bounded_proposals_with_review_batches(self):
        original = deepcopy((self.model, self.spec, self.start))
        with patch("arm_project.arm_link.SerialTransport.open") as opened:
            report = plan_approach(self.model, self.spec, self.start)
        opened.assert_not_called()
        self.assertEqual(report["status"], "OFFLINE_PROPOSAL")
        self.assertEqual(len(report["stages"]), 4)
        self.assertGreater(report["outside_current_window_steps"], 0)
        self.assertEqual(
            sum(b["step_count"] for b in report["review_batches"]), len(report["steps"])
        )
        self.assertTrue(
            all(
                b["step_count"] <= 20 and b["fresh_state_and_review_required"]
                for b in report["review_batches"]
            )
        )
        self.assertEqual((self.model, self.spec, self.start), original)
        for flag in (
            "motion_authorized",
            "physical_target_verified",
            "physical_geometry_verified",
            "base_fixation_verified",
            "obstacles_checked",
        ):
            self.assertFalse(report[flag])
        self.assertEqual(report["held_axes"], [4, 5])

    def test_unreachable_stage_does_not_reuse_an_earlier_solution(self):
        plan_approach(self.model, self.spec, self.start)
        spec = deepcopy(self.spec)
        spec["stages"][0]["pose"]["y_mm"] = 9000
        report = plan_approach(self.model, spec, self.start)
        self.assertEqual(report["status"], "NOMINAL_PATH_BLOCKED")
        self.assertEqual(report["steps"], [])
        self.assertEqual(report["stages"][0]["ik_solution_count"], 0)

    def test_measured_or_duplicate_stage_labels_cannot_masquerade_as_nominal(self):
        for kind in ("source", "verified", "duplicate"):
            spec = deepcopy(self.spec)
            if kind == "source":
                spec["source"] = "HARDWARE_MEASURED"
            elif kind == "verified":
                spec["physical_target_verified"] = True
            else:
                spec["stages"][1]["name"] = spec["stages"][0]["name"]
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                plan_approach(self.model, spec, self.start)

    def pose_file(self):
        # HARDWARE is test metadata only; all values are a synthetic CLI fixture.
        path = self.folder / "pose.json"
        path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "poses": [
                        {
                            "name": "fixture",
                            "positions_p": self.start + [1495, 1514],
                            "source": "HARDWARE",
                            "captured_at": "2026-11-01T02:00:00+08:00",
                            "verified": False,
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        return path

    def test_cli_records_hashes_and_never_opens_device(self):
        path = self.pose_file()
        output = self.folder / "proposal.json"
        with (
            patch("arm_project.arm_link.SerialTransport.open") as opened,
            redirect_stdout(StringIO()),
        ):
            self.assertEqual(main(["--pose-file", str(path), "--output", str(output)]), 0)
        opened.assert_not_called()
        report = json.loads(output.read_text())
        self.assertEqual(set(report["input_sha256"]), {"pose", "model", "spec"})
        self.assertTrue(all(len(h) == 64 for h in report["input_sha256"].values()))
        self.assertFalse(report["motion_authorized"])

    def test_cli_never_overwrites_a_report(self):
        path = self.pose_file()
        output = self.folder / "exists.json"
        output.write_text("preserve", encoding="utf-8")
        with redirect_stdout(StringIO()):
            self.assertEqual(main(["--pose-file", str(path), "--output", str(output)]), 1)
        self.assertEqual(output.read_text(), "preserve")


if __name__ == "__main__":
    unittest.main()
