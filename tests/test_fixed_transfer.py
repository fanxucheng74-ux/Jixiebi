"""固定取放规划仅验证数学和假设，不允许设备 IO。"""

import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.fixed_transfer import plan_fixed_transfer


ROOT = Path(__file__).resolve().parents[1]


def request():
    return {
        "distance_reference": "RADIAL_FROM_BASE",
        "distance_mm": 200,
        "left_yaw_deg": -45,
        "right_yaw_deg": 45,
        "block_height_mm": 25,
        "block_footprint_xy_mm": [25, 25],
        "tip_to_grasp_mm": 0,
        "table_z_mm": 0,
        "approach_clearance_mm": 50,
        "cycle_count": 5,
        "stage_times_ms": {
            "descend_pick": 1000,
            "close": 500,
            "lift_pick": 1000,
            "rotate_to_place": 1200,
            "descend_place": 1000,
            "open": 500,
            "lift_place": 1000,
            "rotate_to_pick": 1200,
        },
    }


class FixedTransferTests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads(
            (ROOT / "configs/kinematics.vendor_nominal.json").read_text(encoding="utf-8")
        )

    def test_radial_coordinates_repeatability_and_no_hardware_authorization(self):
        # 与串口/相机隔离，数学点不能混同于实物闭环结果。
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("port")):
            result = plan_fixed_transfer(self.model, request())
        pick, place = result["points_mm"]["pick"], result["points_mm"]["place"]
        self.assertAlmostEqual(pick[0], -200 / math.sqrt(2))
        self.assertAlmostEqual(place[0], 200 / math.sqrt(2))
        self.assertAlmostEqual(pick[1], 200 / math.sqrt(2))
        self.assertEqual(pick[2], 12.5)
        self.assertEqual(result["block_footprint_xy_mm"], [25.0, 25.0])
        self.assertEqual(result["cycle_count"], 5)
        self.assertEqual(result["declared_ms_per_transfer"], 7400)
        self.assertTrue(result["within_10s_declared_only"])
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["physical_five_cycles_verified"])
        self.assertFalse(result["hardware_interpolation_implemented"])
        self.assertFalse(result["execution_overhead_included"])
        self.assertFalse(result["current_timed_cycle_executor_compatible"])
        self.assertFalse(result["dynamic_vision_selection_implemented"])
        self.assertIn("bounding-box", result["caveat"])
        self.assertIn("new object", result["object_supply"])
        self.assertIn("cleared", result["destination_management"])
        self.assertFalse(result["supply_and_clearing_time_included"])
        self.assertFalse(result["initial_move_to_pick_hover_included"])

    def test_distance_reference_changes_horizontal_position(self):
        data = request()
        data["distance_reference"] = "FORWARD_Y"
        result = plan_fixed_transfer(self.model, data)
        pick = result["points_mm"]["pick"]
        self.assertAlmostEqual(pick[0], -200)
        self.assertAlmostEqual(pick[1], 200)
        self.assertAlmostEqual(math.hypot(*pick[:2]), 200 * math.sqrt(2))

    def test_quintic_preview_endpoints_and_monotonic_time(self):
        result = plan_fixed_transfer(self.model, request())
        rotate = next(x for x in result["stages"] if x["name"] == "rotate_to_place")
        samples = rotate["preview"]["samples"]
        self.assertEqual(samples[0]["pulses_p"], result["nominal_endpoints_p"]["pick_hover"])
        self.assertEqual(samples[-1]["pulses_p"], result["nominal_endpoints_p"]["place_hover"])
        self.assertEqual(samples[0]["time_ms"], 0)
        self.assertEqual(samples[-1]["time_ms"], 1200)
        self.assertTrue(all(a["time_ms"] < b["time_ms"] for a, b in zip(samples, samples[1:])))
        self.assertFalse(rotate["preview"]["physical_path_verified"])

    def test_invalid_or_incomplete_inputs_fail_closed(self):
        for field, value in (
            ("cycle_count", 6),
            ("cycle_count", True),
            ("block_height_mm", float("nan")),
            ("distance_reference", "GUESS"),
        ):
            with self.subTest(field=field, value=value):
                data = request()
                data[field] = value
                with self.assertRaises(ValueError):
                    plan_fixed_transfer(self.model, data)
        data = request()
        del data["tip_to_grasp_mm"]
        with self.assertRaises(ValueError):
            plan_fixed_transfer(self.model, data)

    def test_unreachable_does_not_return_stale_proposal(self):
        plan_fixed_transfer(self.model, request())
        data = request()
        data["distance_mm"] = 1000
        with self.assertRaisesRegex(ValueError, "no nominal"):
            plan_fixed_transfer(self.model, data)

    def test_declared_time_exceeding_goal_is_not_mislabeled(self):
        data = request()
        data["stage_times_ms"]["rotate_to_pick"] = 5000
        result = plan_fixed_transfer(self.model, data)
        self.assertGreater(result["declared_ms_per_transfer"], 10000)
        self.assertFalse(result["within_10s_declared_only"])
        self.assertFalse(result["motion_authorized"])


if __name__ == "__main__":
    unittest.main()
