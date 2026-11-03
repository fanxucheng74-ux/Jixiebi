"""Failed feedback retains exact nominal-scene evidence without hardware access."""

from copy import deepcopy
import json
from pathlib import Path
import unittest

from arm_project.arm_link import ServoLink
from arm_project.motion_scene import SceneValidator
from arm_project.transfer_task_motion import (
    TaskTransport,
    TransferTaskController,
    TransferTaskSpec,
)
from test_arm_driver import FakeClock
from test_timed_cycle import Device
from test_transfer_task_motion import review_payload


ROOT = Path(__file__).resolve().parents[1]
FAILED_POSITIONS = [1820, 1320, 2170, 1200, 1504, 1220]


class _StatefulInspector:
    def __init__(self, result):
        self.calls = []
        self.result = result

    def __call__(self, positions):
        raise AssertionError("inspect-capable validators must not be called a second time")

    def inspect(self, positions):
        self.calls.append(positions)
        return self.result


class TaskSceneDiagnosticsTests(unittest.TestCase):
    def controller(self, validator, *, record=None):
        clock = FakeClock()
        raw = Device(clock)
        spec = TransferTaskSpec.from_dict(review_payload())
        gate = TaskTransport("NEVER_OPENED", spec, raw=raw)
        events = []

        def capture(event, **values):
            if record is not None:
                record(event, **values)
            events.append((event, deepcopy(values)))

        link = ServoLink(gate, clock=clock, record=capture)
        controller = TransferTaskController(
            link, spec, lambda value: None, state_validator=validator
        )
        return controller, raw, events

    def test_inspect_is_called_once_without_calling_stateful_bool_wrapper(self):
        validator = _StatefulInspector({"valid": True})
        controller, raw, events = self.controller(validator)
        controller.guard_geometry([1500] * 6)
        self.assertEqual(validator.calls, [(1500, 1500, 1500, 1500)])
        self.assertEqual(events, [])
        self.assertEqual(raw.writes, [])

    def test_reported_endpoint_rejection_retains_full_scene_and_exact_six_axis_pose(self):
        model = json.loads(
            (ROOT / "configs/kinematics.arduino_nominal.json").read_text(encoding="utf-8")
        )
        scene_config = json.loads(
            (ROOT / "configs/motion_scene.example.json").read_text(encoding="utf-8")
        )
        scene = SceneValidator(model, scene_config)
        controller, raw, events = self.controller(scene)
        controller.link.transport.phase_index = 2
        positions = list(FAILED_POSITIONS)
        with self.assertRaisesRegex(ValueError, "code=TABLE_CLEARANCE.*link_index=2") as caught:
            controller.guard_geometry(positions)
        self.assertIn("minimum_table_clearance_mm=-3.167", str(caught.exception))
        self.assertIn("required_table_clearance_mm=5", str(caught.exception))
        self.assertEqual(len(events), 1)
        event, detail = events[0]
        self.assertEqual(event, "task_scene_rejected")
        self.assertEqual(detail["positions6_p"], FAILED_POSITIONS)
        self.assertEqual(detail["positions4_p"], FAILED_POSITIONS[:4])
        self.assertEqual(detail["phase_index"], 2)
        self.assertEqual(detail["phase_name"], "cycle_01_rotate_left")
        self.assertFalse(detail["simultaneous"])
        self.assertEqual(detail["scene_check"], scene.inspect(FAILED_POSITIONS[:4]))
        positions[1] = 1500
        self.assertEqual(detail["positions6_p"], FAILED_POSITIONS)
        self.assertEqual(raw.writes, [])

    def test_failed_stateful_inspection_is_not_repeated_for_diagnostic_capture(self):
        check = {"valid": False, "code": "RECOVERY_PROGRESS_DECREASED", "links": [{"z": 1}]}
        validator = _StatefulInspector(check)
        controller, _, events = self.controller(validator)
        with self.assertRaisesRegex(ValueError, "RECOVERY_PROGRESS_DECREASED"):
            controller.guard_geometry([1500] * 6)
        self.assertEqual(len(validator.calls), 1)
        detail = events[0][1]
        self.assertEqual(detail["phase_index"], -1)
        self.assertIsNone(detail["phase_name"])
        check["links"][0]["z"] = 99
        self.assertEqual(detail["scene_check"]["links"][0]["z"], 1)

    def test_bool_only_validator_keeps_unknown_scene_unknown(self):
        controller, _, events = self.controller(lambda positions: False)
        with self.assertRaisesRegex(ValueError, "inspection_available=False"):
            controller.guard_geometry([1500] * 6)
        detail = events[0][1]
        self.assertIsNone(detail["scene_check"])
        self.assertFalse(detail["inspection_available"])
        self.assertEqual(detail["validation_return_type"], "bool")

    def test_truthy_nonboolean_inspection_is_still_rejected(self):
        validator = _StatefulInspector({"valid": "yes", "code": "BAD_VALIDATOR"})
        controller, raw, events = self.controller(validator)
        with self.assertRaisesRegex(ValueError, "validation_return_type=str"):
            controller.guard_geometry([1500] * 6)
        self.assertEqual(len(validator.calls), 1)
        self.assertEqual(events[0][1]["scene_check"]["valid"], "yes")
        self.assertEqual(raw.writes, [])

    def test_malformed_inspect_result_does_not_fall_back_to_a_second_observation(self):
        validator = _StatefulInspector(True)
        controller, _, events = self.controller(validator)
        with self.assertRaisesRegex(ValueError, "validation_return_type=NoneType"):
            controller.guard_geometry([1500] * 6)
        self.assertEqual(len(validator.calls), 1)
        self.assertTrue(events[0][1]["inspection_available"])
        self.assertIsNone(events[0][1]["scene_check"])

    def test_diagnostic_log_failure_propagates_to_the_existing_stop_flow(self):
        clock = FakeClock()
        raw = Device(clock)
        spec = TransferTaskSpec.from_dict(review_payload())
        gate = TaskTransport("NEVER_OPENED", spec, raw=raw)
        events = []

        def record(event, **values):
            if event == "task_scene_rejected":
                raise OSError("scene diagnostic storage failed")
            events.append((event, deepcopy(values)))

        link = ServoLink(gate, clock=clock, record=record)
        controller = TransferTaskController(
            link, spec, lambda value: None, state_validator=lambda p: not bool(raw.moves)
        )
        result = controller.run()
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
        self.assertFalse(result["response_pass"])
        self.assertIn("OSError: scene diagnostic storage failed", result["error"])
        self.assertEqual(raw.writes.count(b"#000PDST!"), 1)
        self.assertFalse(any(b"#005P" in frame and b"T" in frame for frame in raw.writes))
        stored = [value["result"] for event, value in events if event == "cycle_result"]
        self.assertEqual(len(stored), 1)
        self.assertEqual(stored[0], result)

    def test_failure_event_and_final_error_survive_the_existing_result_logger(self):
        controller, raw, events = self.controller(
            lambda p: not bool(controller.link.transport.raw.moves)
        )
        result = controller.run()
        self.assertEqual(result["status"], "FAILED_AFTER_ATTEMPT")
        rejections = [value for event, value in events if event == "task_scene_rejected"]
        self.assertEqual(len(rejections), 1)
        self.assertEqual(rejections[0]["phase_name"], "entry")
        # 读完六轴到停止请求之间仍会经过主机时间，比较当次原始观察而非事后设备位置。
        states = [value["state"] for event, value in events if event == "arm_state"]
        observed = [servo["position_p"] for servo in states[-1]["servos"]]
        self.assertEqual(rejections[0]["positions6_p"], observed)
        self.assertFalse(result["response_pass"])
        self.assertEqual(raw.writes.count(b"#000PDST!"), 1)
        stored = [value["result"] for event, value in events if event == "cycle_result"]
        self.assertEqual(stored, [result])


if __name__ == "__main__":
    unittest.main()
