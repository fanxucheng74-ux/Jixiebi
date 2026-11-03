"""ROS 单次夹爪任务的离线行为；FakeDriver 从不打开串口或控制设备。"""

from threading import Event, Thread
import unittest

from arm_project.ros_single_grip import OneShotGrip, SingleGripPlan


PULSES = (1510, 1503, 1495, 1495, 1504, 1514)


def reviewed_plan():
    return {
        "schema_version": 1,
        "review_id": "grip_review_001",
        "evidence": "夹爪空载且工作区经人工检查",
        "action": "open",
        "start_intervals_p": [[p - 2, p + 2] for p in PULSES],
        "reviewed": True,
    }


class FakeDriver:
    source = "HARDWARE"

    def __init__(self):
        self.positions = PULSES
        self.calls = []
        self.cancelled = False
        self.started = Event()
        self.unblock = Event()
        self.block = False
        self.override_state = None
        self.result_sequence = []
        self.result = {
            "status": "SUCCEEDED",
            "response_pass": True,
            "position_reached": True,
            "error": None,
        }

    def read_state(self):
        self.calls.append("read_state")
        if self.override_state is not None:
            return self.override_state
        return {
            "source": "HARDWARE",
            "status": "OK",
            "servos": [
                {"servo_id": index, "status": "OK", "position_p": p}
                for index, p in enumerate(self.positions)
            ],
        }

    def grip(self, action, *, execution_id, step_id):
        self.calls.append(("grip", action, execution_id, step_id))
        self.started.set()
        if self.block and not self.unblock.wait(2):
            raise RuntimeError("test timed out")
        result = self.result_sequence.pop(0) if self.result_sequence else self.result
        return {"execution_id": execution_id, "step_id": step_id, **result}

    def cancel(self):
        self.cancelled = True
        self.unblock.set()


class RosSingleGripTests(unittest.TestCase):
    def test_plan_rejects_invalid_and_unreviewed_inputs(self):
        # 语义、真实性声明和六轴起点必须完整，额外字段也不能变成隐藏指令。
        variants = [
            {"reviewed": False},
            {"action": "raw_command"},
            {"schema_version": True},
            {"evidence": ""},
            {"review_id": ""},
            {"start_intervals_p": [[1500, 1501]]},
            {"start_intervals_p": [[1600, 1500]] * 6},
            {"start_intervals_p": [[True, 1500]] * 6},
            {"unexpected": "#005P1500!"},
        ]
        for mutation in variants:
            with self.subTest(mutation=mutation), self.assertRaises((ValueError, TypeError)):
                SingleGripPlan.from_dict({**reviewed_plan(), **mutation})

        # 直接调用数据类构造器也必须经过同样的review和字段检查。
        intervals = tuple((p - 2, p + 2) for p in PULSES)
        with self.assertRaises(ValueError):
            SingleGripPlan("review_1", "", "open", intervals, True)
        with self.assertRaises(ValueError):
            SingleGripPlan("review_1", "real observation", "open", intervals, False)

    def test_valid_plan_executes_once_only(self):
        plan = SingleGripPlan.from_dict(reviewed_plan())
        driver = FakeDriver()
        executor = OneShotGrip(plan, driver)
        first = executor.execute()
        self.assertIsInstance(first, dict)
        self.assertIn(first["status"], ("SUCCEEDED", "RESPONSE_OBSERVED"))
        self.assertEqual(sum(isinstance(call, tuple) for call in driver.calls), 1)
        try:
            again = executor.execute()
        except RuntimeError:
            pass
        else:
            self.assertNotIn(again["status"], ("SUCCEEDED", "RESPONSE_OBSERVED"))
        self.assertEqual(sum(isinstance(call, tuple) for call in driver.calls), 1)

    def test_position_outside_reviewed_start_never_moves(self):
        driver = FakeDriver()
        driver.positions = (PULSES[0] + 10, *PULSES[1:])
        executor = OneShotGrip(SingleGripPlan.from_dict(reviewed_plan()), driver)
        try:
            result = executor.execute()
        except (ValueError, RuntimeError):
            pass
        else:
            self.assertNotIn(result["status"], ("SUCCEEDED", "RESPONSE_OBSERVED"))
        self.assertFalse(any(isinstance(call, tuple) for call in driver.calls))

    def test_driver_failure_is_terminal_and_not_resent(self):
        driver = FakeDriver()
        driver.result = {
            "status": "FAILED",
            "response_pass": False,
            "position_reached": False,
            "error": "通信中断",
        }
        executor = OneShotGrip(SingleGripPlan.from_dict(reviewed_plan()), driver)
        try:
            result = executor.execute()
        except (ValueError, RuntimeError):
            pass
        else:
            self.assertNotIn(result["status"], ("SUCCEEDED", "RESPONSE_OBSERVED"))
        try:
            executor.execute()
        except (ValueError, RuntimeError):
            pass
        self.assertEqual(sum(isinstance(call, tuple) for call in driver.calls), 1)

    def test_mismatched_identity_and_inconsistent_status_are_rejected(self):
        for mutation in (
            {"execution_id": "different_execution"},
            {"step_id": "wrong_step"},
            {"status": "FAILED", "response_pass": True, "error": None},
            {"status": "SUCCEEDED", "response_pass": True, "position_reached": False},
        ):
            with self.subTest(mutation=mutation):
                driver = FakeDriver()
                driver.result.update(mutation)
                executor = OneShotGrip(SingleGripPlan.from_dict(reviewed_plan()), driver)
                result = executor.execute()
                self.assertEqual(result["status"], "FAILED")
                self.assertEqual(sum(isinstance(call, tuple) for call in driver.calls), 1)

    def test_open_close_continues_on_valid_response_without_claiming_exact_arrival(self):
        driver = FakeDriver()
        driver.result_sequence = [
            {
                "status": "RESPONSE_OBSERVED",
                "response_pass": True,
                "position_reached": False,
                "error": None,
            },
            {"status": "SUCCEEDED", "response_pass": True, "position_reached": True, "error": None},
        ]
        plan = SingleGripPlan.from_dict({**reviewed_plan(), "action": "open_close"})
        result = OneShotGrip(plan, driver).execute()
        self.assertEqual(result["status"], "RESPONSE_OBSERVED")
        self.assertEqual(
            [call[1] for call in driver.calls if isinstance(call, tuple)], ["open", "close"]
        )

    def test_invalid_or_simulated_state_cannot_authorize_motion(self):
        normal = FakeDriver().read_state()
        variants = [
            {**normal, "source": "SIMULATION"},
            {**normal, "status": "INCOMPLETE"},
            {**normal, "servos": normal["servos"][:-1]},
            {**normal, "servos": list(reversed(normal["servos"]))},
            {
                **normal,
                "servos": [{**normal["servos"][0], "status": "READ_FAILED"}, *normal["servos"][1:]],
            },
        ]
        for state in variants:
            with self.subTest(state=state):
                driver = FakeDriver()
                driver.override_state = state
                executor = OneShotGrip(SingleGripPlan.from_dict(reviewed_plan()), driver)
                try:
                    result = executor.execute()
                except (ValueError, RuntimeError):
                    pass
                else:
                    self.assertNotIn(result["status"], ("SUCCEEDED", "RESPONSE_OBSERVED"))
                self.assertFalse(any(isinstance(call, tuple) for call in driver.calls))

    def test_cancel_before_start_prevents_motion(self):
        driver = FakeDriver()
        executor = OneShotGrip(SingleGripPlan.from_dict(reviewed_plan()), driver)
        executor.cancel()
        try:
            result = executor.execute()
        except RuntimeError:
            pass
        else:
            self.assertNotIn(result["status"], ("SUCCEEDED", "RESPONSE_OBSERVED"))
        self.assertTrue(driver.cancelled)
        self.assertFalse(any(isinstance(call, tuple) for call in driver.calls))

    def test_concurrent_call_rejected_and_cancel_reaches_driver(self):
        driver = FakeDriver()
        driver.block = True
        executor = OneShotGrip(SingleGripPlan.from_dict(reviewed_plan()), driver)
        outcomes = []
        worker = Thread(target=lambda: outcomes.append(executor.execute()), daemon=True)
        worker.start()
        self.assertTrue(driver.started.wait(2), "driver.grip 未开始")
        try:
            result = executor.execute()
        except RuntimeError:
            pass
        else:
            self.assertNotIn(result["status"], ("SUCCEEDED", "RESPONSE_OBSERVED"))
        executor.cancel()
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertTrue(driver.cancelled)
        self.assertEqual(sum(isinstance(call, tuple) for call in driver.calls), 1)
        self.assertEqual(len(outcomes), 1)
        self.assertNotIn(outcomes[0]["status"], ("SUCCEEDED", "RESPONSE_OBSERVED"))

    def test_cancel_during_result_validation_does_not_report_success(self):
        # 在驱动已返回、结果正在核对时取消，不能同时给出取消成功和执行成功。
        driver = FakeDriver()
        validating = Event()

        class DelayedResult(dict):
            delayed = False

            def get(self, key, default=None):
                if key == "status" and not self.delayed:
                    self.delayed = True
                    validating.set()
                    if not driver.unblock.wait(2):
                        raise RuntimeError("test timed out")
                return super().get(key, default)

        def delayed_grip(action, *, execution_id, step_id):
            driver.calls.append(("grip", action, execution_id, step_id))
            return DelayedResult(execution_id=execution_id, step_id=step_id, **driver.result)

        driver.grip = delayed_grip
        executor = OneShotGrip(SingleGripPlan.from_dict(reviewed_plan()), driver)
        outcomes = []
        worker = Thread(target=lambda: outcomes.append(executor.execute()), daemon=True)
        worker.start()
        self.assertTrue(validating.wait(2), "结果校验尚未开始")
        self.assertEqual(executor.cancel()["status"], "CANCELLED")
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(outcomes[0]["status"], "CANCELLED")
        self.assertEqual(sum(isinstance(call, tuple) for call in driver.calls), 1)


if __name__ == "__main__":
    unittest.main()
