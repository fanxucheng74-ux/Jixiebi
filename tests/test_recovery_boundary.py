"""Nominal recovery must finish before gripper work or a successful task result."""

import json
from pathlib import Path
import unittest

from arm_project.arm_link import ServoLink
from arm_project.clearance_recovery import task_scene_validator
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_task_motion import (
    TaskTransport,
    TransferTaskController,
    TransferTaskSpec,
    motion_phases_sha256,
)
from test_arm_driver import FakeClock
from test_timed_cycle import Device
from test_transfer_task_motion import phase, review_payload


ROOT = Path(__file__).resolve().parents[1]
START = [1833, 1331, 2163, 1190, 1500, 1200]
ENTRY_GOAL = [1833, 1338, 2163, 1190]


class _ResidualDevice(Device):
    def __init__(self, clock, residual):
        super().__init__(clock)
        self.initial = list(START)
        self.residual = residual
        self.degraded = False

    def position(self, sid):
        if sid == 1 and sid in self.moves:
            if self.degraded:
                return 1336
            start, target, duration, when = self.moves[sid]
            fraction = min(1, (self.clock.now - when) / (duration / 1000))
            return round(start + (target + self.residual - start) * fraction)
        return super().position(sid)


class RecoveryBoundaryTests(unittest.TestCase):
    def run_task(self, *, residual=-2, prepare=True, degrade=False):
        model = json.loads(
            (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
        )
        scene_config = json.loads(
            (ROOT / "configs/motion_scene.example.json").read_text(encoding="utf-8")
        )
        scene = SceneValidator(model, scene_config)
        validator = task_scene_validator(
            scene, {"current_positions6_p": START, "entry_route4_p": [START[:4], ENTRY_GOAL]}
        )
        review = review_payload()
        review.update(
            start_intervals_p=[[p - 2, p + 2] for p in START],
            observation_bounds_p=[[500, 2500] for _ in range(6)],
            geometry_mm=model["geometry_mm"],
            deadline_ms=5000,
            phases=[phase("entry_01", {1: 1338}, 1500, 20)],
        )
        if prepare:
            review["phases"].append(phase("prepare_gripper", {5: 1200}, 1000, 20))
        review["motion_phases_sha256"] = motion_phases_sha256(review["phases"])
        clock = FakeClock()
        raw = _ResidualDevice(clock, residual)
        events = []

        def record(event, **values):
            # 模拟已通过入口普通场景后重新下沉，必须由普通规则拒绝，不能再开恢复例外。
            if degrade and event == "cycle_phase_end" and values["phase"] == "entry_01":
                raw.degraded = True
            events.append((event, json.loads(json.dumps(values))))

        spec = TransferTaskSpec.from_dict(review)
        gate = TaskTransport("NEVER_OPENED", spec, raw=raw)
        link = ServoLink(gate, clock=clock, record=record)
        controller = TransferTaskController(link, spec, lambda p: None, state_validator=validator)
        return controller.run(), raw, validator, scene, events

    def test_two_p_entry_residual_blocks_gripper_before_its_first_frame(self):
        result, raw, validator, _, events = self.run_task()
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT", result["error"])
        self.assertFalse(result["response_pass"])
        self.assertIn("recovery has not reached", result["error"])
        self.assertFalse(validator.normal_scene_active)
        self.assertFalse(any(frame.startswith(b"#005P") and b"T" in frame for frame in raw.writes))
        self.assertEqual(raw.writes.count(b"#001PDST!"), 1)
        logged = [values["result"] for event, values in events if event == "cycle_result"]
        self.assertEqual(len(logged), 1)
        self.assertFalse(logged[0]["response_pass"])
        self.assertEqual(logged[0]["status"], "FAILED_AFTER_ATTEMPT")

    def test_entry_only_task_cannot_complete_inside_the_temporary_margin(self):
        result, raw, validator, scene, events = self.run_task(prepare=False)
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT", result["error"])
        self.assertFalse(result["response_pass"])
        self.assertFalse(validator.normal_scene_active)
        final = [servo["position_p"] for servo in result["final_state"]["servos"]]
        self.assertEqual(final[1], 1336)
        self.assertAlmostEqual(scene.inspect(final[:4])["minimum_table_clearance_mm"], 4.5126269547)
        self.assertEqual(raw.writes.count(b"#001PDST!"), 1)
        self.assertFalse(
            any(
                event == "cycle_result" and values["result"]["response_pass"]
                for event, values in events
            )
        )

    def test_observed_normal_recovery_allows_preparation_and_success(self):
        result, raw, validator, _, _ = self.run_task(residual=0)
        self.assertTrue(result["response_pass"], result["error"])
        self.assertEqual(result["status"], "RESPONSE_SEQUENCE_COMPLETED")
        self.assertTrue(validator.normal_scene_active)
        self.assertIn(b"#005P1200T1000!", raw.writes)
        self.assertFalse(any(frame.endswith(b"PDST!") for frame in raw.writes))

    def test_degradation_after_latching_normal_rules_still_rejects(self):
        result, raw, validator, _, _ = self.run_task(residual=0, degrade=True)
        self.assertFalse(result["response_pass"])
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT", result["error"])
        self.assertTrue(validator.normal_scene_active)
        self.assertEqual(validator.last_check["code"], "TABLE_CLEARANCE")
        self.assertEqual(raw.writes.count(b"#001PDST!"), 1)


if __name__ == "__main__":
    unittest.main()
