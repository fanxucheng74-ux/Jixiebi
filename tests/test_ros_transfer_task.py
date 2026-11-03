"""有限任务ROS适配离线测试；ROS和运行器均为桩，不连接任何设备。"""

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
from threading import Event, Thread
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = ROOT / "ros2_ws/src/jixiebi_driver/jixiebi_driver/transfer_task.py"
EVENTS = []


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


class FakeNode:
    def __init__(self, name):
        self.node_name = name
        self.services = {}
        self.logs = []
        EVENTS.append("node_init")

    def create_publisher(self, message_type, topic, qos):
        self.output_topic, self.output_qos = topic, qos
        return FakePublisher()

    def create_service(self, service_type, topic, callback, *, callback_group):
        service = types.SimpleNamespace(callback=callback, callback_group=callback_group)
        self.services[topic] = service
        return service

    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda text: self.logs.append(("info", text)),
            error=lambda text: self.logs.append(("error", text)),
        )

    def destroy_node(self):
        EVENTS.append("node_destroy")


class FakeRunner:
    instances = []
    init_error = False

    def __init__(self, config, port, progress=None):
        EVENTS.append("runner_init")
        if self.init_error:
            raise ValueError("invalid task config")
        self.config, self.port, self.progress = config, port, progress
        self.run_calls = self.cancel_calls = 0
        self.used = self.finished = self.block = False
        self.cancel_event, self.started = Event(), Event()
        self.result = {
            "status": "RESPONSE_SEQUENCE_COMPLETED",
            "response_pass": True,
            "physical_task_verified": False,
            "run_dir": "runs/synthetic_task_01",
        }
        self.error = None
        self.instances.append(self)

    def run(self):
        self.run_calls += 1
        if self.used:
            return {"status": "ALREADY_USED", "response_pass": False}
        self.used = True
        EVENTS.append("run_started")
        self.started.set()
        if self.block and not self.cancel_event.wait(2):
            raise RuntimeError("test did not cancel blocked runner")
        if self.error is not None:
            raise self.error
        self.finished = True
        EVENTS.append("run_finished")
        if self.cancel_event.is_set():
            return {"status": "CANCELLED", "response_pass": False}
        return deepcopy(self.result)

    def cancel(self):
        self.cancel_calls += 1
        EVENTS.append("cancel_requested")
        self.cancel_event.set()
        return {"status": "FINISHED" if self.finished else "CANCEL_REQUESTED"}


def load_node_module():
    rclpy = types.ModuleType("rclpy")
    ros_state = {"ok": False, "init_args": None}

    def init(*, args):
        EVENTS.append("ros_init")
        ros_state.update(ok=True, init_args=args)

    def shutdown():
        EVENTS.append("ros_shutdown")
        ros_state["ok"] = False

    rclpy.init, rclpy.shutdown = init, shutdown
    rclpy.ok = lambda: ros_state["ok"]
    rclpy.test_state = ros_state
    node = types.ModuleType("rclpy.node")
    node.Node = FakeNode
    groups = types.ModuleType("rclpy.callback_groups")
    groups.MutuallyExclusiveCallbackGroup = type("CallbackGroup", (), {})
    executors = types.ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = object
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="TRANSIENT_LOCAL")
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="RELIABLE")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="KEEP_LAST")
    messages = types.ModuleType("std_msgs.msg")
    messages.String = type("FakeString", (), {})
    services = types.ModuleType("std_srvs.srv")
    services.Trigger = type("FakeTrigger", (), {})
    runtime = types.ModuleType("arm_project.transfer_task_runtime")
    runtime.TransferTaskRunner = FakeRunner
    modules = {
        "rclpy": rclpy,
        "rclpy.node": node,
        "rclpy.callback_groups": groups,
        "rclpy.executors": executors,
        "rclpy.qos": qos,
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": messages,
        "std_srvs": types.ModuleType("std_srvs"),
        "std_srvs.srv": services,
        "arm_project.transfer_task_runtime": runtime,
    }
    spec = importlib.util.spec_from_file_location("transfer_task_ros_stub_test", NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class TransferTaskROSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_module()

    def setUp(self):
        EVENTS.clear()
        FakeRunner.instances.clear()
        FakeRunner.init_error = False
        self.module.rclpy.test_state.update(ok=False, init_args=None)

    def node(self):
        with patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")):
            return self.module.TransferTaskNode({"test_only": "unreviewed"}, "TEST-NO-DEVICE")

    def response(self):
        return types.SimpleNamespace(success=None, message=None)

    def test_waiting_node_has_no_run_and_distinct_start_cancel_groups(self):
        node = self.node()
        self.assertEqual(node.node_name, "jixiebi_transfer_task")
        self.assertEqual(node.runner.run_calls, 0)
        self.assertEqual(node.runner.cancel_calls, 0)
        self.assertEqual(node.runner.port, "TEST-NO-DEVICE")
        self.assertEqual(set(node.services), {"/jixiebi/task/start", "/jixiebi/task/cancel"})
        self.assertIsNot(node.start_group, node.cancel_group)
        self.assertEqual(node.output_topic, "/jixiebi/task/status")
        self.assertEqual(node.output_qos.depth, 1)
        self.assertEqual(node.output_qos.reliability, "RELIABLE")
        self.assertEqual(node.output_qos.durability, "TRANSIENT_LOCAL")
        initial = node.publisher.messages[-1]
        self.assertEqual(initial["status"], "READY_FOR_REQUEST")
        self.assertEqual(initial["source"], "ROS_TASK_WRAPPER")
        self.assertFalse(initial["motion_authorized"])
        self.assertFalse(initial["motion_started"])

    def test_success_requires_exact_completed_status_and_boolean_response_pass(self):
        for status, passed, expected in (
            ("RESPONSE_SEQUENCE_COMPLETED", True, True),
            ("RESPONSE_SEQUENCE_COMPLETED", False, False),
            ("RESPONSE_SEQUENCE_COMPLETED", 1, False),
            ("FAILED", True, False),
            ("REVIEW_REQUIRED", True, False),
            ("ENTRY_BLOCKED", True, False),
            ("CANCELLED", True, False),
        ):
            with self.subTest(status=status, passed=passed):
                node = self.node()
                node.runner.result.update(status=status, response_pass=passed)
                result = node.on_start(object(), self.response())
                self.assertIs(result.success, expected)
                self.assertEqual(json.loads(result.message), node.publisher.messages[-1])
                self.assertEqual(node.runner.run_calls, 1)

    def test_duplicate_start_is_not_retried_and_completed_cancel_is_not_success(self):
        node = self.node()
        self.assertTrue(node.on_start(object(), self.response()).success)
        again = node.on_start(object(), self.response())
        self.assertFalse(again.success)
        self.assertEqual(json.loads(again.message)["status"], "ALREADY_USED")
        self.assertEqual(EVENTS.count("run_started"), 1)
        cancelled = node.on_cancel(object(), self.response())
        self.assertFalse(cancelled.success)
        self.assertEqual(json.loads(cancelled.message)["status"], "FINISHED")

    def test_progress_preserves_source_and_logs_runtime_evidence_paths(self):
        node = self.node()
        progress = {
            "status": "PHASE_COMPLETE",
            "source": "SIMULATION",
            "run_dir": "runs/synthetic_task_02",
            "raw_log_path": "runs/synthetic_task_02/raw.jsonl",
            "physical_task_verified": False,
        }
        node.runner.progress(progress)
        self.assertEqual(node.publisher.messages[-1], progress)
        self.assertTrue(any("raw.jsonl" in text for _, text in node.logs))
        self.assertNotIn("HARDWARE", json.dumps(node.publisher.messages))

    def test_cancel_only_signals_existing_runner_and_can_interrupt_start(self):
        node = self.node()
        node.runner.block = True
        outcomes = []
        worker = Thread(target=lambda: outcomes.append(node.on_start(object(), self.response())))
        worker.start()
        self.assertTrue(node.runner.started.wait(2))
        cancel = node.on_cancel(object(), self.response())
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertTrue(cancel.success)
        self.assertEqual(json.loads(cancel.message)["status"], "CANCEL_REQUESTED")
        self.assertEqual(node.runner.cancel_calls, 1)
        self.assertEqual(node.runner.run_calls, 1)
        self.assertFalse(outcomes[0].success)
        self.assertEqual(json.loads(outcomes[0].message)["status"], "CANCELLED")

    def test_unexpected_runtime_error_is_failed_and_never_retried(self):
        node = self.node()
        node.runner.error = RuntimeError("synthetic runtime failure")
        result = node.on_start(object(), self.response())
        self.assertFalse(result.success)
        self.assertEqual(json.loads(result.message)["status"], "FAILED")
        self.assertEqual(node.runner.run_calls, 1)
        self.assertEqual(node.runner.cancel_calls, 1)
        self.assertTrue(any(level == "error" for level, _ in node.logs))

    def test_main_rejects_duplicate_json_and_requires_both_cli_arguments(self):
        for raw in ('{"review": 1, "review": 2}', '{"nested": {"p": 1, "p": 2}}', "[]"):
            with (
                self.subTest(raw=raw),
                patch.object(self.module.Path, "read_text", return_value=raw),
                self.assertRaises(ValueError),
            ):
                self.module.main(["--config", "unused.json", "--port", "TEST-NO-DEVICE"])
        self.assertNotIn("ros_init", EVENTS)
        self.assertEqual(FakeRunner.instances, [])
        for args in (["--config", "unused.json"], ["--port", "TEST-NO-DEVICE"]):
            with self.subTest(args=args), patch("sys.stderr"), self.assertRaises(SystemExit):
                self.module.main(args)

    def test_main_cancels_and_waits_for_worker_before_destroy(self):
        testcase = self

        class Executor:
            def __init__(self, num_threads):
                testcase.assertEqual(num_threads, 2)
                self.worker = None

            def add_node(self, node):
                self.node = node

            def spin(self):
                self.node.runner.block = True
                self.worker = Thread(
                    target=lambda: self.node.on_start(object(), testcase.response())
                )
                self.worker.start()
                testcase.assertTrue(self.node.runner.started.wait(2))
                raise KeyboardInterrupt

            def shutdown(self, *, wait_for_threads):
                EVENTS.append("executor_shutdown")
                testcase.assertIs(wait_for_threads, True)
                self.worker.join(2)
                testcase.assertFalse(self.worker.is_alive())
                EVENTS.append("executor_workers_finished")

        with (
            patch.object(self.module, "MultiThreadedExecutor", Executor),
            patch.object(self.module.Path, "read_text", return_value='{"test_only": true}'),
            patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")),
        ):
            self.module.main(["--config", "unused.json", "--port", "TEST-NO-DEVICE", "--ros-args"])
        self.assertLess(EVENTS.index("cancel_requested"), EVENTS.index("executor_shutdown"))
        self.assertLess(EVENTS.index("run_finished"), EVENTS.index("executor_workers_finished"))
        self.assertLess(EVENTS.index("executor_workers_finished"), EVENTS.index("node_destroy"))
        self.assertLess(EVENTS.index("node_destroy"), EVENTS.index("ros_shutdown"))
        self.assertEqual(self.module.rclpy.test_state["init_args"], ["--ros-args"])

    def test_runner_config_error_cleans_up_without_starting(self):
        FakeRunner.init_error = True

        class Executor:
            def __init__(self, num_threads):
                pass

            def shutdown(self, *, wait_for_threads):
                EVENTS.append("executor_shutdown")

        with (
            patch.object(self.module, "MultiThreadedExecutor", Executor),
            patch.object(self.module.Path, "read_text", return_value="{}"),
            patch("arm_project.arm_link.SerialTransport.open", side_effect=AssertionError("IO")),
            self.assertRaises(ValueError),
        ):
            self.module.main(["--config", "unused.json", "--port", "TEST-NO-DEVICE"])
        self.assertNotIn("run_started", EVENTS)
        self.assertIn("node_destroy", EVENTS)
        self.assertIn("executor_shutdown", EVENTS)
        self.assertEqual(EVENTS[-1], "ros_shutdown")


if __name__ == "__main__":
    unittest.main()
