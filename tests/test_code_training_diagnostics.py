"""Synchronized boundaries tested with fake CUDA; never import an ML runtime."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call

from tools.code_training_diagnostics import training_operation


def fake_torch():
    cuda = SimpleNamespace(
        synchronize=Mock(),
        mem_get_info=Mock(return_value=(3 * 2**20, 16 * 2**20)),
        memory_allocated=Mock(return_value=5 * 2**20),
        memory_reserved=Mock(return_value=8 * 2**20),
        max_memory_allocated=Mock(return_value=7 * 2**20),
        memory_stats=Mock(
            return_value={
                "active_bytes.all.current": 4 * 2**20,
                "inactive_split_bytes.all.current": 2 * 2**20,
            }
        ),
    )
    return SimpleNamespace(cuda=cuda)


class TrainingOperationTests(unittest.TestCase):
    def test_disabled_boundary_never_accesses_torch_or_logs(self):
        event = Mock()
        body = Mock()
        with training_operation(None, event, "forward"):
            body()
        body.assert_called_once_with()
        event.assert_not_called()

    def test_enabled_boundary_synchronizes_and_records_memory_and_context(self):
        torch = fake_torch()
        event = Mock()
        body = Mock()
        with training_operation(torch, event, "backward", enabled=True, sample_id="one"):
            self.assertEqual(torch.cuda.synchronize.call_count, 1)
            self.assertEqual(event.call_args.kwargs["phase"], "ready")
            body()
        body.assert_called_once_with()
        self.assertEqual(torch.cuda.synchronize.call_args_list, [call(0), call(0)])
        self.assertEqual(
            [row.kwargs["phase"] for row in event.call_args_list], ["start", "ready", "complete"]
        )
        for row in event.call_args_list:
            self.assertEqual(row.args, ("backward",))
            self.assertEqual(row.kwargs["sample_id"], "one")
        memory = event.call_args.kwargs["memory"]
        self.assertEqual(memory["allocated_mib"], 5)
        self.assertEqual(memory["reserved_mib"], 8)
        self.assertEqual(memory["peak_allocated_mib"], 7)
        self.assertEqual(memory["global_free_mib"], 3)
        self.assertEqual(memory["active_mib"], 4)
        self.assertEqual(memory["inactive_split_mib"], 2)

    def test_body_exception_is_preserved_without_completion_event(self):
        torch = fake_torch()
        event = Mock()
        error = RuntimeError("forward kernel failed")
        with self.assertRaises(RuntimeError) as caught:
            with training_operation(torch, event, "forward", enabled=True):
                raise error
        self.assertIs(caught.exception, error)
        self.assertEqual(torch.cuda.synchronize.call_count, 1)
        self.assertEqual([row.kwargs["phase"] for row in event.call_args_list], ["start", "ready"])

    def test_initial_sync_failure_prevents_body_and_records_started_stage(self):
        torch = fake_torch()
        error = RuntimeError("context lost")
        torch.cuda.synchronize.side_effect = error
        event = Mock()
        body = Mock()
        with self.assertRaises(RuntimeError) as caught:
            with training_operation(torch, event, "batch_alloc", enabled=True):
                body()
        self.assertIs(caught.exception, error)
        body.assert_not_called()
        event.assert_called_once_with("batch_alloc", phase="start")

    def test_final_sync_failure_never_records_completion(self):
        torch = fake_torch()
        error = RuntimeError("async kernel failed")
        torch.cuda.synchronize.side_effect = [None, error]
        event = Mock()
        body = Mock()
        with self.assertRaises(RuntimeError) as caught:
            with training_operation(torch, event, "backward", enabled=True):
                body()
        self.assertIs(caught.exception, error)
        body.assert_called_once_with()
        self.assertEqual([row.kwargs["phase"] for row in event.call_args_list], ["start", "ready"])


if __name__ == "__main__":
    unittest.main()
