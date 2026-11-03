"""Offline, bounded command deduplication and connection recovery.

No socket, Wi-Fi, serial, filesystem, or motion access is performed here. A
caller explicitly supplies the executor. A handshake is read-only; ``arm`` is
an explicit upper-layer decision, never an automatic reconnect action.

The ledger is in memory, not a crash-durable exactly-once mechanism. Every
process/device restart MUST use a new boot_id. Never retry an UNKNOWN command
under a new ID automatically. Disconnecting does not physically stop an arm.
"""

from collections import OrderedDict
from dataclasses import dataclass, field
from enum import Enum
import json
import math
from threading import Lock
from typing import Any, Callable, Optional
from uuid import uuid4


# 限制账本内规范化命令正文和结果正文的 UTF-8 字节数；网络帧另有上层限制。
MAX_COMMAND_BYTES = 1024
MAX_RESULT_BYTES = 4096
# JSON 可能由 JavaScript 等端处理；2^53-1 内的整数可被其数值类型精确表示。
MAX_SEQUENCE = (1 << 53) - 1


class CommandRejected(ValueError):
    """The request was rejected without invoking an executor."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


class CommandState(str, Enum):
    # UNKNOWN 是“效果无法确认”，不是“肯定没有执行”；不能据此自动重发运动。
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class SessionToken:
    # boot_id 区分设备/进程的启动代次；generation 区分同会话的前后两次连接。
    boot_id: str
    session_id: str
    generation: int


@dataclass(frozen=True)
class CommandRequest:
    # sequence 单调递增用于拒绝过期请求；command_id 是本会话内的命令身份。
    # operation/payload 是待交给 executor 的业务名称和参数，而非直接串口指令。
    boot_id: str
    session_id: str
    sequence: int
    command_id: str
    operation: str
    payload: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CommandStatus:
    # 状态与结果分开保存；结果可能因容量限制被淘汰，但身份和执行状态仍保留。
    boot_id: str
    session_id: str
    sequence: int
    command_id: str
    state: CommandState
    error: Optional[str] = None
    reused: bool = False
    _result_json: Optional[str] = field(default=None, repr=False)

    @property
    def result_available(self) -> bool:
        return self._result_json is not None

    @property
    def result(self) -> Any:
        """Return a detached value, so callers cannot corrupt a cached result."""
        # 每次从 JSON 重建新对象；调用方修改返回的字典不会污染缓存中的结果。
        return json.loads(self._result_json) if self.result_available else None

    def to_dict(self) -> dict[str, Any]:
        return {
            "boot_id": self.boot_id,
            "session_id": self.session_id,
            "sequence": self.sequence,
            "command_id": self.command_id,
            "state": self.state.value,
            "error": self.error,
            "reused": self.reused,
            "result_available": self.result_available,
            "result": self.result,
        }


@dataclass
class _Session:
    # connected 是连接状态，armed 是上层明确允许提交新命令；握手不自动获得许可。
    generation: int = 1
    connected: bool = True
    armed: bool = False
    last_sequence: int = 0


@dataclass
class _Entry:
    # fingerprint 保存规范化后的参数文本，用来发现“同一 ID 却换了参数”的冲突。
    sequence: int
    fingerprint: str
    state: CommandState = CommandState.RUNNING
    error: Optional[str] = None


def _text(value: Any, name: str, limit: int = 64) -> str:
    if type(value) is not str or not value.strip() or len(value) > limit:
        raise CommandRejected("INVALID_REQUEST", name + " must be a bounded nonempty string")
    return value


def _json_value(value: Any, depth: int = 0) -> None:
    # 递归限制 JSON 的类型、深度及有限数值，避免异常对象或无穷嵌套进入账本。
    if depth > 16:
        raise CommandRejected("INVALID_REQUEST", "JSON nesting exceeds 16 levels")
    if value is None or type(value) in (str, int, bool):
        return
    if type(value) is float and math.isfinite(value):
        return
    if type(value) is list:
        for item in value:
            _json_value(item, depth + 1)
        return
    if type(value) is dict and all(type(key) is str for key in value):
        for item in value.values():
            _json_value(item, depth + 1)
        return
    raise CommandRejected("INVALID_REQUEST", "only finite JSON values and string keys are allowed")


def _canonical(value: Any, limit: int) -> str:
    _json_value(value)
    try:
        # 键排序和紧凑格式使字典插入顺序不同但内容相同的命令具有相同指纹。
        encoded = json.dumps(
            value, sort_keys=True, ensure_ascii=False, allow_nan=False, separators=(",", ":")
        )
        length = len(encoded.encode("utf-8"))
    except (ValueError, TypeError, UnicodeError, RecursionError) as error:
        raise CommandRejected("INVALID_REQUEST", "invalid JSON value") from error
    if length > limit:
        raise CommandRejected("INVALID_REQUEST", "JSON value exceeds byte limit")
    return encoded


class CommandLedger:
    """One executor at a time, exact deduplication within a bounded boot epoch.

    Result bodies have an LRU bound. Small identity/fingerprint/state records
    are retained separately, up to max_history. When history is full, NEW
    commands fail closed; records are never silently evicted or reused. This
    preserves deduplication after result eviction without unbounded memory.
    Session records are also bounded and never silently recycled.

    All sessions control the same arm. Any RUNNING-to-UNKNOWN transition
    disarms every session. Each must explicitly arm again after upper-layer
    reconciliation; an executor still in flight prevents all rearming.
    """

    def __init__(
        self,
        *,
        boot_id: Optional[str] = None,
        result_capacity: int = 64,
        max_history: int = 256,
        max_sessions: int = 8,
    ):
        self.boot_id = _text(uuid4().hex if boot_id is None else boot_id, "boot_id")
        for name, value in (
            ("result_capacity", result_capacity),
            ("max_history", max_history),
            ("max_sessions", max_sessions),
        ):
            if type(value) is not int or value < 1:
                raise ValueError(name + " must be a positive integer")
        if result_capacity > max_history:
            raise ValueError("result_capacity cannot exceed max_history")
        self._capacity = result_capacity
        self._history_limit = max_history
        self._session_limit = max_sessions
        self._lock = Lock()
        # 多个连接共享一条机械臂；这些表的检查与状态变更要在同一把锁下完成。
        self._sessions: dict[str, _Session] = {}
        # key=(session_id, command_id)；启动身份由当前 ledger 的 boot_id 统一约束。
        self._entries: dict[tuple[str, str], _Entry] = {}
        # _results 仅缓存结果正文，按最近访问顺序淘汰；_entries 的身份记录不随之删除。
        self._results: OrderedDict[tuple[str, str], str] = OrderedDict()
        # _active 持有当前 executor 对应的 key；非 None 时其他命令不能并发执行。
        self._active: Optional[tuple[str, str]] = None

    def handshake(self, session_id: str, *, expected_boot_id: Optional[str] = None) -> SessionToken:
        """Register a new connection generation, always unarmed and read-only.

        Repeating a handshake replaces the prior connection; it is not an
        executor retry. A RUNNING command on that connection becomes UNKNOWN.
        """
        _text(session_id, "session_id")
        if expected_boot_id is not None and expected_boot_id != self.boot_id:
            raise CommandRejected("BOOT_MISMATCH", "the device boot epoch changed")
        with self._lock:
            if session_id in self._sessions:
                # 同 ID 再握手表示替换旧连接；旧令牌随 generation 递增而失效。
                self._invalidate_active(session_id, "connection replaced during execution")
                session = self._sessions[session_id]
                session.generation += 1
                session.connected, session.armed = True, False
            else:
                if len(self._sessions) >= self._session_limit:
                    raise CommandRejected("SESSION_CAPACITY", "session history is full")
                session = self._sessions[session_id] = _Session()
            return SessionToken(self.boot_id, session_id, session.generation)

    def _session(self, token: SessionToken, *, connected: bool = True) -> _Session:
        # 每次操作都核对启动身份和连接代次，防止旧连接持有的令牌继续控制设备。
        if not isinstance(token, SessionToken) or token.boot_id != self.boot_id:
            raise CommandRejected("BOOT_MISMATCH", "token belongs to another boot epoch")
        session = self._sessions.get(token.session_id)
        if (
            session is None
            or type(token.generation) is not int
            or token.generation != session.generation
        ):
            raise CommandRejected("STALE_CONNECTION", "connection token has been replaced")
        if connected and not session.connected:
            raise CommandRejected("DISCONNECTED", "handshake is required before querying state")
        return session

    def arm(self, token: SessionToken) -> None:
        """Explicitly permit new commands after upper-layer reconciliation.

        This does not execute, retry, clear UNKNOWN records, reset sequence
        numbers, or establish mechanical safety. A new connection is never
        armed automatically, even if an older generation was armed.
        Any session's UNKNOWN outcome disarms all sessions. Rearming remains
        BUSY until the old executor exits, then only this session is armed.
        """
        with self._lock:
            session = self._session(token)
            if self._active is not None:
                # 即使已经掉线，旧 executor 未返回前仍占用执行权，不能重新放行。
                raise CommandRejected("BUSY", "an earlier executor has not returned")
            session.armed = True

    def disconnect(self, token: SessionToken) -> None:
        """Invalidate pending outcomes; never execute a stop or replay callback."""
        with self._lock:
            session = self._session(token, connected=False)
            session.connected, session.armed = False, False
            self._invalidate_active(token.session_id, "connection lost during execution")

    def _invalidate_active(self, session_id: str, reason: str) -> None:
        if self._active is not None and self._active[0] == session_id:
            self._mark_unknown(self._entries[self._active], reason)
        # The executor still owns the single dispatch slot until it returns.

    def _mark_unknown(self, entry: _Entry, reason: str) -> None:
        """Called under the lock: one uncertain arm outcome invalidates all permits."""
        if entry.state == CommandState.RUNNING:
            entry.state, entry.error = CommandState.UNKNOWN, reason
            # 所有会话控制同一机械臂；任何一次效果不明，都撤销其他会话的新命令许可。
            for session in self._sessions.values():
                session.armed = False

    def _status(self, key: tuple[str, str], *, reused: bool = False) -> CommandStatus:
        entry = self._entries[key]
        body = self._results.get(key)
        if body is not None:
            # 命中后移到队尾，保留最近被查询的结果；只改变缓存顺序，不再执行命令。
            self._results.move_to_end(key)
        return CommandStatus(
            self.boot_id, key[0], entry.sequence, key[1], entry.state, entry.error, reused, body
        )

    def query(self, token: SessionToken, command_id: str) -> CommandStatus:
        _text(command_id, "command_id")
        with self._lock:
            self._session(token)
            # 查询仅访问已有记录；没有记录不能推断那条命令从未产生物理效果。
            key = (token.session_id, command_id)
            if key not in self._entries:
                raise CommandRejected(
                    "UNKNOWN_COMMAND", "no record for this session and command ID"
                )
            return self._status(key)

    def dispatch(
        self,
        token: SessionToken,
        request: CommandRequest,
        executor: Callable[[str, dict[str, Any]], Any],
    ) -> CommandStatus:
        """Execute a new accepted command once; duplicate calls only return state.

        The executor gets a detached payload. Any exception or unrepresentable
        result is UNKNOWN, since effects may have occurred. That ID is never
        dispatched again. KeyboardInterrupt/SystemExit are recorded and then
        propagated. No automatic retry or rearming is provided.
        """
        if not isinstance(request, CommandRequest) or not callable(executor):
            raise CommandRejected("INVALID_REQUEST", "a request and callable executor are required")
        for name in ("boot_id", "session_id", "command_id", "operation"):
            _text(getattr(request, name), name)
        if type(request.sequence) is not int or not 1 <= request.sequence <= MAX_SEQUENCE:
            raise CommandRejected("INVALID_REQUEST", "sequence must be a positive exact integer")
        if type(request.payload) is not dict:
            raise CommandRejected("INVALID_REQUEST", "payload must be a JSON object")
        # 指纹包含序号、操作和参数；command_id 本身作为字典 key 的一部分保存。
        fingerprint = _canonical(
            {
                "sequence": request.sequence,
                "operation": request.operation,
                "payload": request.payload,
            },
            MAX_COMMAND_BYTES,
        )
        # 反序列化得到独立的参数副本，调用方随后修改原 payload 不会改掉本次请求。
        detached = json.loads(fingerprint)
        key = (request.session_id, request.command_id)
        with self._lock:
            session = self._session(token)
            if request.boot_id != self.boot_id:
                raise CommandRejected("BOOT_MISMATCH", "old-boot commands cannot be replayed")
            if request.session_id != token.session_id:
                raise CommandRejected("SESSION_MISMATCH", "command belongs to a different session")
            old = self._entries.get(key)
            if old is not None:
                # 重复请求只返回缓存状态；若同 ID 内容不同则拒绝，不覆盖原记录。
                if old.fingerprint != fingerprint:
                    raise CommandRejected(
                        "ID_CONFLICT", "same command ID has different sequence or arguments"
                    )
                return self._status(key, reused=True)
            if not session.armed:
                raise CommandRejected(
                    "NOT_ARMED", "handshake/reconnection only permits state queries"
                )
            if request.sequence <= session.last_sequence:
                # 序号一旦消费就不回退；换一个新 command_id 也不能重用旧序号。
                raise CommandRejected(
                    "STALE_SEQUENCE", "sequence was already consumed or superseded"
                )
            if self._active is not None:
                raise CommandRejected("BUSY", "only one executor may run at a time")
            if len(self._entries) >= self._history_limit:
                # 身份记录满时拒绝新命令，不能为了腾空间删除去重证据。
                raise CommandRejected(
                    "HISTORY_CAPACITY", "deduplication history is full; do not discard it"
                )
            # 先原子登记 RUNNING 和已消费序号，再调用可能实际改变机械臂状态的函数。
            self._entries[key] = _Entry(request.sequence, fingerprint)
            session.last_sequence = request.sequence
            self._active = key  # Registered under lock before invoking the executor.

        try:
            # executor 在锁外运行，避免耗时动作挡住查询/断连；_active 仍禁止并发执行。
            result = executor(request.operation, detached["payload"])
            body = _canonical(result, MAX_RESULT_BYTES)
        except BaseException as error:
            # 异常可能发生在动作已经开始之后，所以记 UNKNOWN，而不是自动重试。
            try:
                detail = type(error).__name__ + ": " + str(error)[:256]
            except BaseException:
                detail = type(error).__name__
            with self._lock:
                entry = self._entries[key]
                self._mark_unknown(entry, detail)
                self._active = None
                outcome = self._status(key)
            if not isinstance(error, Exception):
                # 用户中断等退出信号记录后继续向上传播，不被当成普通业务错误吞掉。
                raise
            return outcome
        with self._lock:
            entry = self._entries[key]
            if entry.state == CommandState.RUNNING:
                entry.state = CommandState.COMPLETED
                self._results[key] = body
                while len(self._results) > self._capacity:
                    # 只淘汰最久未访问的结果正文；状态/指纹仍在 _entries，可继续去重。
                    self._results.popitem(last=False)
            # A completion arriving after disconnect does not erase UNKNOWN.
            self._active = None
            return self._status(key)

    def snapshot(self) -> dict[str, Any]:
        # 这是账本的只读统计快照，不含机械臂当前姿态，也不表示状态已落盘持久化。
        with self._lock:
            return {
                "boot_id": self.boot_id,
                "history_count": len(self._entries),
                "result_count": len(self._results),
                "session_count": len(self._sessions),
                "executor_in_flight": self._active is not None,
                "persistence": "MEMORY_ONLY",
                "hardware_access": False,
            }
