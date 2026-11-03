"""Pure-memory command/session tests: no network, serial, camera, or hardware."""

from dataclasses import replace
from threading import Event, Thread
import unittest

from arm_project.command_ledger import (
    CommandLedger,
    CommandRejected,
    CommandRequest,
    CommandState,
)


class LedgerTests(unittest.TestCase):
    def setUp(self):
        self.ledger = CommandLedger(boot_id="boot-a", result_capacity=2, max_history=6)
        self.token = self.ledger.handshake("session-a", expected_boot_id="boot-a")
        self.calls = []

    def request(self, number=1, **changes):
        req = CommandRequest(
            "boot-a",
            "session-a",
            number,
            "cmd-" + str(number),
            "MOVE_LOCAL",
            {"servo_id": 4, "target_p": 1540},
        )
        return replace(req, **changes)

    def execute(self, operation, payload):
        self.calls.append((operation, payload))
        return {"response_pass": True, "position_reached": False}

    def reject(self, code, function, *args):
        with self.assertRaises(CommandRejected) as raised:
            function(*args)
        self.assertEqual(raised.exception.code, code)

    def test_handshake_is_read_only_until_explicit_arm(self):
        self.reject("NOT_ARMED", self.ledger.dispatch, self.token, self.request(), self.execute)
        self.assertEqual(self.calls, [])
        self.ledger.arm(self.token)
        state = self.ledger.dispatch(self.token, self.request(), self.execute)
        self.assertEqual(state.state, CommandState.COMPLETED)
        self.assertTrue(state.result["response_pass"])
        self.assertFalse(state.result["position_reached"])

    def test_duplicate_and_lost_return_execute_only_once(self):
        self.ledger.arm(self.token)
        self.ledger.dispatch(self.token, self.request(), self.execute)  # Client loses this return.
        cached = self.ledger.dispatch(self.token, self.request(), self.execute)
        self.assertEqual(len(self.calls), 1)
        self.assertTrue(cached.reused)
        cached.result["response_pass"] = False
        self.assertTrue(self.ledger.query(self.token, "cmd-1").result["response_pass"])

    def test_id_collision_rejects_changed_operation_sequence_or_payload(self):
        self.ledger.arm(self.token)
        self.ledger.dispatch(self.token, self.request(), self.execute)
        for changed in (
            self.request(operation="OTHER"),
            self.request(sequence=2),
            self.request(payload={"servo_id": 4, "target_p": 1490}),
        ):
            self.reject("ID_CONFLICT", self.ledger.dispatch, self.token, changed, self.execute)
        self.assertEqual(len(self.calls), 1)

    def test_canonical_payload_order_deduplicates_and_executor_cannot_mutate_request(self):
        self.ledger.arm(self.token)
        req = self.request(payload={"a": [1], "b": 2})

        def mutate(operation, payload):
            self.calls.append(operation)
            payload["a"].append(9)
            return {"ok": True}

        self.ledger.dispatch(self.token, req, mutate)
        self.assertEqual(req.payload, {"a": [1], "b": 2})
        same = replace(req, payload={"b": 2, "a": [1]})
        self.assertTrue(self.ledger.dispatch(self.token, same, mutate).reused)
        self.assertEqual(len(self.calls), 1)

    def test_registered_before_dispatch_and_concurrent_duplicate_never_executes(self):
        self.ledger.arm(self.token)
        entered, release = Event(), Event()
        results, errors = [], []

        def slow(operation, payload):
            self.calls.append(operation)
            self.assertEqual(self.ledger.query(self.token, "cmd-1").state, CommandState.RUNNING)
            entered.set()
            if not release.wait(3):
                raise RuntimeError("test release timeout")
            return {"done": True}

        def work():
            try:
                results.append(self.ledger.dispatch(self.token, self.request(), slow))
            except BaseException as error:
                errors.append(error)

        thread = Thread(target=work)
        thread.start()
        try:
            self.assertTrue(entered.wait(3))
            duplicate = self.ledger.dispatch(self.token, self.request(), self.execute)
            self.assertEqual(duplicate.state, CommandState.RUNNING)
            self.assertTrue(duplicate.reused)
            self.reject("BUSY", self.ledger.dispatch, self.token, self.request(2), self.execute)
            self.assertEqual(self.ledger.snapshot()["history_count"], 1)
        finally:
            release.set()
            thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(results[0].state, CommandState.COMPLETED)

    def test_executor_exception_is_unknown_and_never_automatically_retried(self):
        self.ledger.arm(self.token)

        def effects_then_fail(operation, payload):
            self.calls.append(operation)
            raise OSError("outcome not known")

        first = self.ledger.dispatch(self.token, self.request(), effects_then_fail)
        self.assertEqual(first.state, CommandState.UNKNOWN)
        self.assertIn("OSError", first.error)
        self.assertEqual(
            self.ledger.dispatch(self.token, self.request(), self.execute).state,
            CommandState.UNKNOWN,
        )
        self.reject("NOT_ARMED", self.ledger.dispatch, self.token, self.request(2), self.execute)
        self.ledger.arm(self.token)
        self.assertEqual(
            self.ledger.dispatch(self.token, self.request(), self.execute).state,
            CommandState.UNKNOWN,
        )
        self.assertEqual(len(self.calls), 1)

    def test_exception_disarms_other_previously_armed_sessions(self):
        other = self.ledger.handshake("session-b")
        self.ledger.arm(self.token)
        self.ledger.arm(other)
        other_old = self.request(session_id="session-b")
        other_new = self.request(2, session_id="session-b")
        self.ledger.dispatch(other, other_old, self.execute)

        def effects_then_fail(operation, payload):
            self.calls.append(operation)
            raise OSError("uncertain arm state")

        state = self.ledger.dispatch(self.token, self.request(), effects_then_fail)
        self.assertEqual(state.state, CommandState.UNKNOWN)
        self.reject("NOT_ARMED", self.ledger.dispatch, other, other_new, self.execute)
        self.reject("NOT_ARMED", self.ledger.dispatch, self.token, self.request(2), self.execute)
        self.assertEqual(self.ledger.query(other, "cmd-1").state, CommandState.COMPLETED)
        self.assertTrue(self.ledger.dispatch(other, other_old, self.execute).reused)
        self.assertEqual(
            self.ledger.dispatch(self.token, self.request(), self.execute).state,
            CommandState.UNKNOWN,
        )
        self.assertEqual(len(self.calls), 2)
        self.ledger.arm(other)
        self.assertEqual(
            self.ledger.dispatch(other, other_new, self.execute).state,
            CommandState.COMPLETED,
        )
        self.reject("NOT_ARMED", self.ledger.dispatch, self.token, self.request(2), self.execute)
        self.assertEqual(len(self.calls), 3)

    def _assert_connection_loss_disarms_all_sessions(self, *, replace_connection):
        other = self.ledger.handshake("session-b")
        self.ledger.arm(self.token)
        self.ledger.arm(other)
        other_request = self.request(session_id="session-b")
        entered, release = Event(), Event()
        outcomes, errors = [], []

        def slow(operation, payload):
            self.calls.append(operation)
            entered.set()
            if not release.wait(3):
                raise RuntimeError("test release timeout")
            return {"late": True}

        def work():
            try:
                outcomes.append(self.ledger.dispatch(self.token, self.request(), slow))
            except BaseException as error:
                errors.append(error)

        thread = Thread(target=work)
        thread.start()
        try:
            self.assertTrue(entered.wait(3))
            if not replace_connection:
                self.ledger.disconnect(self.token)
            restored = self.ledger.handshake("session-a")
            self.assertEqual(self.ledger.query(restored, "cmd-1").state, CommandState.UNKNOWN)
            self.assertEqual(
                self.ledger.dispatch(restored, self.request(), self.execute).state,
                CommandState.UNKNOWN,
            )
            self.reject("NOT_ARMED", self.ledger.dispatch, other, other_request, self.execute)
            self.reject("BUSY", self.ledger.arm, other)
            self.reject("BUSY", self.ledger.arm, restored)
            self.assertEqual(len(self.calls), 1)
        finally:
            release.set()
            thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(outcomes[0].state, CommandState.UNKNOWN)
        self.reject("NOT_ARMED", self.ledger.dispatch, other, other_request, self.execute)
        self.reject("NOT_ARMED", self.ledger.dispatch, restored, self.request(2), self.execute)
        self.ledger.arm(other)
        self.assertEqual(
            self.ledger.dispatch(other, other_request, self.execute).state,
            CommandState.COMPLETED,
        )
        self.reject("NOT_ARMED", self.ledger.dispatch, restored, self.request(2), self.execute)
        self.assertEqual(len(self.calls), 2)

    def test_disconnect_disarms_other_previously_armed_sessions(self):
        self._assert_connection_loss_disarms_all_sessions(replace_connection=False)

    def test_connection_replacement_disarms_other_previously_armed_sessions(self):
        self._assert_connection_loss_disarms_all_sessions(replace_connection=True)

    def test_keyboard_interrupt_is_recorded_and_propagated(self):
        self.ledger.arm(self.token)

        def interrupted(operation, payload):
            raise KeyboardInterrupt()

        with self.assertRaises(KeyboardInterrupt):
            self.ledger.dispatch(self.token, self.request(), interrupted)
        self.assertEqual(self.ledger.query(self.token, "cmd-1").state, CommandState.UNKNOWN)
        self.assertFalse(self.ledger.snapshot()["executor_in_flight"])

    def test_invalid_result_preserves_unknown_no_retry(self):
        self.ledger.arm(self.token)
        state = self.ledger.dispatch(self.token, self.request(), lambda *_: float("nan"))
        self.assertEqual(state.state, CommandState.UNKNOWN)
        self.assertFalse(state.result_available)
        self.ledger.dispatch(self.token, self.request(), self.execute)
        self.assertEqual(self.calls, [])

    def test_result_eviction_preserves_exact_deduplication_tombstone(self):
        self.ledger.arm(self.token)
        for n in range(1, 4):
            self.ledger.dispatch(self.token, self.request(n), self.execute)
        old = self.ledger.query(self.token, "cmd-1")
        self.assertEqual(old.state, CommandState.COMPLETED)
        self.assertFalse(old.result_available)
        self.assertTrue(self.ledger.dispatch(self.token, self.request(), self.execute).reused)
        self.reject(
            "ID_CONFLICT", self.ledger.dispatch, self.token, self.request(sequence=4), self.execute
        )
        self.assertEqual(len(self.calls), 3)
        self.assertEqual(self.ledger.snapshot()["result_count"], 2)

    def test_full_history_rejects_new_commands_without_discarding_old_ids(self):
        ledger = CommandLedger(boot_id="boot-a", result_capacity=1, max_history=2)
        token = ledger.handshake("session-a")
        ledger.arm(token)
        for n in (1, 2):
            ledger.dispatch(token, self.request(n), self.execute)
        self.reject("HISTORY_CAPACITY", ledger.dispatch, token, self.request(3), self.execute)
        self.assertTrue(ledger.dispatch(token, self.request(), self.execute).reused)
        self.assertEqual(len(self.calls), 2)

    def test_sequence_is_monotonic_even_with_another_command_id(self):
        self.ledger.arm(self.token)
        self.ledger.dispatch(self.token, self.request(2), self.execute)
        self.reject(
            "STALE_SEQUENCE", self.ledger.dispatch, self.token, self.request(), self.execute
        )
        self.reject(
            "STALE_SEQUENCE",
            self.ledger.dispatch,
            self.token,
            self.request(2, command_id="another-id"),
            self.execute,
        )

    def test_reconnect_queries_old_result_without_replaying_or_arming(self):
        self.ledger.arm(self.token)
        self.ledger.dispatch(self.token, self.request(), self.execute)
        self.ledger.disconnect(self.token)
        self.reject("DISCONNECTED", self.ledger.query, self.token, "cmd-1")
        restored = self.ledger.handshake("session-a", expected_boot_id="boot-a")
        self.reject("STALE_CONNECTION", self.ledger.query, self.token, "cmd-1")
        self.assertEqual(self.ledger.query(restored, "cmd-1").state, CommandState.COMPLETED)
        self.assertTrue(self.ledger.dispatch(restored, self.request(), self.execute).reused)
        self.reject("NOT_ARMED", self.ledger.dispatch, restored, self.request(2), self.execute)
        self.assertEqual(len(self.calls), 1)

    def test_disconnect_during_execution_preserves_unknown_and_serial_ownership(self):
        self.ledger.arm(self.token)
        entered, release = Event(), Event()
        outcomes = []

        def slow(operation, payload):
            self.calls.append(operation)
            entered.set()
            release.wait(3)
            return {"late": True}

        thread = Thread(
            target=lambda: outcomes.append(self.ledger.dispatch(self.token, self.request(), slow))
        )
        thread.start()
        try:
            self.assertTrue(entered.wait(3))
            self.ledger.disconnect(self.token)
            restored = self.ledger.handshake("session-a")
            self.assertEqual(self.ledger.query(restored, "cmd-1").state, CommandState.UNKNOWN)
            self.reject("BUSY", self.ledger.arm, restored)
        finally:
            release.set()
            thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(outcomes[0].state, CommandState.UNKNOWN)
        self.ledger.arm(restored)
        self.assertEqual(
            self.ledger.dispatch(restored, self.request(), self.execute).state, CommandState.UNKNOWN
        )
        self.assertEqual(len(self.calls), 1)

    def test_new_handshake_alone_invalidates_prior_generation(self):
        self.ledger.arm(self.token)
        restored = self.ledger.handshake("session-a")
        self.reject(
            "STALE_CONNECTION", self.ledger.dispatch, self.token, self.request(), self.execute
        )
        self.reject("NOT_ARMED", self.ledger.dispatch, restored, self.request(), self.execute)

    def test_restart_rejects_old_boot_and_never_restores_old_execution(self):
        self.ledger.arm(self.token)
        self.ledger.dispatch(self.token, self.request(), self.execute)
        restarted = CommandLedger(boot_id="boot-b")
        with self.assertRaises(CommandRejected) as raised:
            restarted.handshake("session-a", expected_boot_id="boot-a")
        self.assertEqual(raised.exception.code, "BOOT_MISMATCH")
        token = restarted.handshake("session-a")
        restarted.arm(token)
        self.reject("BOOT_MISMATCH", restarted.dispatch, token, self.request(), self.execute)
        self.reject("BOOT_MISMATCH", restarted.query, self.token, "cmd-1")
        self.assertEqual(len(self.calls), 1)

    def test_cross_session_and_capacity_are_rejected(self):
        ledger = CommandLedger(boot_id="boot-a", max_sessions=1)
        token = ledger.handshake("session-a")
        self.reject("SESSION_CAPACITY", ledger.handshake, "session-b")
        ledger.arm(token)
        self.reject(
            "SESSION_MISMATCH",
            ledger.dispatch,
            token,
            self.request(session_id="session-b"),
            self.execute,
        )

    def test_invalid_request_rejected_before_record_or_execution(self):
        self.ledger.arm(self.token)
        for changes in (
            {"sequence": True},
            {"sequence": 0},
            {"command_id": ""},
            {"payload": {"x": float("nan")}},
            {"payload": {1: "bad key"}},
            {"payload": {"x": b"bytes"}},
            {"payload": {"x": "中" * 1000}},
        ):
            self.reject(
                "INVALID_REQUEST",
                self.ledger.dispatch,
                self.token,
                self.request(**changes),
                self.execute,
            )
        self.assertEqual(self.ledger.snapshot()["history_count"], 0)
        self.assertEqual(self.calls, [])

    def test_unknown_command_lookup_does_not_dispatch(self):
        self.reject("UNKNOWN_COMMAND", self.ledger.query, self.token, "missing")
        self.assertEqual(self.calls, [])


if __name__ == "__main__":
    unittest.main()
