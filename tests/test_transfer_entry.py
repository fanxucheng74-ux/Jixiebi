"""入口规划只验证名义算法与状态门禁，默认不打开串口或相机。"""

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.transfer_entry import plan_simulated_transfer_entry, plan_transfer_entry


ROOT = Path(__file__).resolve().parents[1]
BOOT = "12345678-1234-5678-1234-567812345678"


def policy():
    return {
        "max_age_s": 3.0,
        "at_start_tolerance_p": 5,
        "high_pose_tolerance_p": 8,
        "minimum_tip_height_above_table_mm": 50,
        "minimum_centerline_height_mm": 5,
        "maximum_radial_mm": 400,
        "duration_ms": 3000,
        "max_joint_delta_p": 900,
    }


def state_for(positions):
    # 明确测试替身：不是复制现场日志，时间和HARDWARE字段只用于状态校验器测试。
    return {
        "schema": "jixiebi.raw_state.v1",
        "session_id": "test-session",
        "batch_index": 1,
        "state": {
            "source": "HARDWARE",
            "status": "OK",
            "error": None,
            "position_unit": "protocol_P",
            "simultaneous": False,
            "timestamp_origin": "host_receive",
            "host_boot_id": BOOT,
            "finished_monotonic_s": 100.7,
            "servos": [
                {
                    "servo_id": sid,
                    "status": "OK",
                    "position_p": p,
                    "received_monotonic_s": 100.1 + sid * 0.1,
                }
                for sid, p in enumerate(positions)
            ],
        },
    }


class EntryPlannerTests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads(
            (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
        )
        request = json.loads(
            (ROOT / "configs/fixed_transfer_nominal_example.json").read_text(encoding="utf-8")
        )
        self.plan = plan_fixed_transfer(self.model, request)
        self.right = self.plan["nominal_endpoints_p"]["place_hover"] + [1500, 1500]
        self.left = self.plan["nominal_endpoints_p"]["pick_hover"] + [1500, 1500]

    def live(self, positions=None, payload=None, rules=None, **kwargs):
        return plan_transfer_entry(
            self.model,
            self.plan,
            state_for(positions or self.right) if payload is None else payload,
            policy() if rules is None else rules,
            now_monotonic_s=101,
            host_boot_id=BOOT,
            expected_session_id="test-session",
            **kwargs,
        )

    def test_right_hover_to_left_proposal_is_bounded_and_never_authorized(self):
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            result = self.live()
        self.assertEqual(result["status"], "NOMINAL_ENTRY_PROPOSAL")
        self.assertEqual(result["source"], "HARDWARE_STATE")
        self.assertTrue(result["hardware_sample_freshness_checked"])
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["physical_path_verified"])
        self.assertFalse(result["hardware_interpolation_implemented"])
        self.assertEqual(len(result["samples"]), 65)
        self.assertEqual(result["samples"][0]["positions4_p"], self.right[:4])
        self.assertEqual(result["samples"][-1]["positions4_p"], self.left[:4])
        self.assertEqual(result["samples"][-1]["time_ms"], 3000)
        self.assertEqual(result["entry_and_transfer_declared_ms"], 10400)
        self.assertFalse(result["within_10s_total_declared_only"])
        self.assertFalse(result["task_ready_verified"])
        self.assertFalse(result["initial_gripper_preparation_included"])
        self.assertTrue(
            all(row["positions4_p"][1:] == self.right[1:4] for row in result["samples"])
        )
        self.assertEqual(result["held_axes_without_new_targets"], {"4": 1500, "5": 1500})

    def test_already_at_left_needs_no_nominal_entry_motion(self):
        result = self.live(self.left)
        self.assertEqual(result["status"], "ALREADY_AT_START")
        self.assertEqual(result["samples"], [])
        self.assertEqual(result["entry_declared_ms"], 0)
        self.assertEqual(result["entry_and_transfer_declared_ms"], 7400)
        self.assertFalse(result["gripper_open_state_verified"])

    def test_low_pose_requires_reviewed_lift_without_guessing_targets(self):
        low = self.plan["nominal_endpoints_p"]["place"] + [1500, 1500]
        result = self.live(low)
        self.assertEqual(result["status"], "REQUIRES_REVIEWED_LIFT")
        self.assertEqual(result["samples"], [])
        self.assertFalse(result["motion_authorized"])

    def test_unknown_high_pose_is_not_arbitrarily_homed(self):
        result = self.live([1500] * 6)
        self.assertEqual(result["status"], "REQUIRES_POSE_REVIEW")
        self.assertEqual(result["samples"], [])

    def test_stale_future_cross_boot_wrong_session_and_invalid_records_rejected(self):
        for edit in (
            lambda p: p["state"]["servos"][0].update(received_monotonic_s=97),
            lambda p: p["state"]["servos"][-1].update(received_monotonic_s=102),
            lambda p: p["state"].update(host_boot_id="aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa"),
            lambda p: p.update(session_id="old-session"),
            lambda p: p["state"].update(source="REPLAY"),
            lambda p: p["state"]["servos"][3].update(servo_id=2),
            lambda p: p["state"]["servos"][2].update(position_p=True),
            lambda p: p["state"]["servos"][0].update(status="NOT_READ"),
            lambda p: p["state"].pop("host_boot_id"),
        ):
            payload = state_for(self.right)
            edit(payload)
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.live(payload=payload)

    def test_model_mismatch_and_untrusted_target_rejected(self):
        for key, value in (("model_geometry_mm", {}), ("motion_authorized", True)):
            old = self.plan
            self.plan = deepcopy(old)
            self.plan[key] = value
            with self.assertRaises(ValueError):
                self.live()
            self.plan = old

    def test_joint_distance_and_sampled_corridor_limits_remain_enforced(self):
        rules = policy()
        rules["max_joint_delta_p"] = 100
        result = self.live(rules=rules)
        self.assertEqual(result["status"], "NOMINAL_PATH_BLOCKED")
        self.assertEqual(result["samples"], [])
        rules = policy()
        rules["maximum_radial_mm"] = 50
        self.assertEqual(self.live(rules=rules)["status"], "NOMINAL_PATH_BLOCKED")

    def test_simulated_entry_has_separate_provenance_and_no_fake_freshness(self):
        result = plan_simulated_transfer_entry(self.model, self.plan, self.right, policy())
        self.assertEqual(result["status"], "NOMINAL_ENTRY_PROPOSAL")
        self.assertEqual(result["source"], "SIMULATION")
        self.assertIsNone(result["freshness"])
        self.assertFalse(result["hardware_sample_freshness_checked"])
        self.assertFalse(result["motion_authorized"])

    def test_invalid_policy_and_pulse_data_rejected(self):
        for field, value in (("duration_ms", True), ("max_age_s", float("nan"))):
            rules = policy()
            rules[field] = value
            with self.assertRaises(ValueError):
                self.live(rules=rules)
        rules = policy()
        del rules["minimum_centerline_height_mm"]
        with self.assertRaises(ValueError):
            self.live(rules=rules)
        with self.assertRaises(ValueError):
            plan_simulated_transfer_entry(self.model, self.plan, [1500] * 4, policy())


if __name__ == "__main__":
    unittest.main()
