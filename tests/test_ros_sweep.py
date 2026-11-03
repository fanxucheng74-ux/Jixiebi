"""仅测试决策逻辑，替身不会连接串口或机械臂。"""

import threading
import unittest

from arm_project.ros_sweep import OneShotSweep, parse_review


def review():
    return {
        "sweep": {
            "review_id": "example",
            "whole_path_reviewed_for_motion": True,
            "evidence": "测试替身",
            "servo_id": 0,
            "min_p": 1400,
            "max_p": 1600,
            "start_min_p": 1490,
            "start_max_p": 1510,
            "target_p": 1550,
            "duration_ms": 3000,
            "max_step_p": 60,
        },
        "start_ranges_p": {str(i): [1490, 1510] for i in range(6)},
    }


class Reader:
    def read_state(self):
        return {
            "source": "HARDWARE",
            "status": "OK",
            "servos": [{"servo_id": i, "status": "OK", "position_p": 1500} for i in range(6)],
        }


class Controller:
    def __init__(self):
        self.calls = 0
        self.cancelled = False

    def run(self):
        self.calls += 1
        return {
            "status": "RESPONSE_OBSERVED",
            "response_pass": True,
            "position_reached": False,
            "error": None,
        }

    def cancel(self):
        self.cancelled = True


class RosSweepTests(unittest.TestCase):
    def test_review_validation(self):
        data = review()
        spec, ranges = parse_review(data)
        assert spec.servo_id == 0 and ranges["5"] == (1490, 1510)
        data["sweep"]["review_id"] = "../bad"
        try:
            parse_review(data)
        except ValueError:
            pass
        else:
            raise AssertionError("路径穿越 review_id 未被拒绝")
        data = review()
        del data["start_ranges_p"]["3"]
        try:
            parse_review(data)
        except ValueError:
            pass
        else:
            raise AssertionError("不完整起始区间未被拒绝")

    def test_once_and_preflight(self):
        spec, ranges = parse_review(review())
        ctrl = Controller()
        runner = OneShotSweep(spec, ranges, None, ctrl, Reader())
        assert runner.execute()["status"] == "RESPONSE_OBSERVED"
        assert runner.execute()["status"] == "ALREADY_USED"
        assert ctrl.calls == 1
        ranges["3"] = (1510, 1520)
        ctrl = Controller()
        runner = OneShotSweep(spec, ranges, None, ctrl, Reader())
        assert runner.execute()["status"] == "FAILED"
        assert ctrl.calls == 0

    def test_invalid_response_is_not_success(self):
        spec, ranges = parse_review(review())
        ctrl = Controller()
        ctrl.run = lambda: {"status": "RESPONSE_OBSERVED", "response_pass": False}
        runner = OneShotSweep(spec, ranges, None, ctrl, Reader())
        assert runner.execute()["status"] == "FAILED"
        ctrl.run = lambda: {
            "status": "TARGET_REACHED",
            "response_pass": True,
            "position_reached": False,
        }
        assert OneShotSweep(spec, ranges, None, ctrl, Reader()).execute()["status"] == "FAILED"

    def test_cancel_during_motion_and_reentry(self):
        spec, ranges = parse_review(review())
        entered, release = threading.Event(), threading.Event()
        ctrl = Controller()

        def run():
            entered.set()
            assert release.wait(2)
            return {"status": "RESPONSE_OBSERVED", "response_pass": True}

        ctrl.run = run
        runner = OneShotSweep(spec, ranges, None, ctrl, Reader())
        out = []
        thread = threading.Thread(target=lambda: out.append(runner.execute()))
        thread.start()
        assert entered.wait(2)
        assert runner.execute()["status"] == "ALREADY_USED"
        assert runner.cancel()["status"] == "CANCEL_REQUESTED"
        release.set()
        thread.join(2)
        assert out[0]["status"] == "CANCELLED"
        assert ctrl.cancelled and not out[0]["physical_stop_verified"]
