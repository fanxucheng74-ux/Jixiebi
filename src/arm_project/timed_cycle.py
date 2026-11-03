"""One reviewed, finite phase sequence; no human pauses or automatic retries.

The declared servo times are not a velocity-continuity guarantee. Periodic
all-axis reads and end-of-phase active-axis checks remain mandatory. Completion
means control responses, never an independently verified paper transfer.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import re

from arm_project.approach_planning import chain_points, is_nominal_upright_reference
from arm_project.arm_link import SerialTransport
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.arm_state import ArmStateReader
from arm_project.joint_probe import JointProbeController
from arm_project.kinematics import Geometry, from_nominal_pulses
from arm_project.servo_protocol import make_move_command, validate_read_query


# 本模块把一整趟有限动作编排为phase序列；串口字节仍交给ArmLink发送/记录。
# 名义几何、协议P反馈和独立物体夹持是不同证据，不由response_pass互相替代。
def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError("integer outside reviewed interval")
    return value


@dataclass(frozen=True)
class Phase:
    # duration_ms为本段发送给舵机的T；dwell_ms是已审阅的段内停留，不是临时人工暂停。
    # targets的每项依次为(舵机ID, 目标P, 阶段交接容差P)，不是XYZ或角度数组。
    name: str
    duration_ms: int
    dwell_ms: int
    targets: tuple
    # hold表示本段仍发送同一已预定目标；continue表示本段不发该轴新帧，只继续监测旧目标。
    hold_tracking_p: tuple = ()
    continue_tracking_p: tuple = ()

    @classmethod
    def from_dict(cls, value):
        required = {
            "name",
            "duration_ms",
            "dwell_ms",
            "targets",
        }
        if (
            not isinstance(value, dict)
            or required - set(value)
            or set(value) - required - {"hold_tracking_p", "continue_tracking_p"}
        ):
            raise ValueError("invalid phase schema")
        if not isinstance(value["name"], str) or not re.fullmatch(
            r"[a-z0-9_]{1,60}", value["name"]
        ):
            raise ValueError("invalid phase name")
        if not isinstance(value["targets"], list):
            raise ValueError("phase targets must be a list")
        targets = []
        for item in value["targets"]:
            if not isinstance(item, dict) or set(item) != {
                "servo_id",
                "target_p",
                "end_tolerance_p",
            }:
                raise ValueError("invalid target schema")
            targets.append(
                (
                    integer(item["servo_id"], 1, 5),
                    integer(item["target_p"], 500, 2500),
                    integer(item["end_tolerance_p"], 3, 80),
                )
            )
        # 夹爪005单独占一个阶段，避免与臂身下探同时开合而无法区分接触/姿态问题。
        ids = [t[0] for t in targets]
        if not 1 <= len(ids) <= 3 or len(set(ids)) != len(ids) or (5 in ids and len(ids) > 1):
            raise ValueError("distinct axes; gripper must have its own phase")
        holds = value.get("hold_tracking_p", {})
        if not isinstance(holds, dict) or any(k not in ("1", "2", "3", "4") for k in holds):
            raise ValueError("explicit body-axis hold tracking map required")
        holds = tuple((int(k), integer(v, 3, 80)) for k, v in holds.items())
        if any(sid not in ids for sid, _ in holds):
            raise ValueError("held setpoint must be an explicit phase target")
        # JSON键为字符串，验证后转整数ID；继续跟踪轴不能又出现在本段targets中。
        continuing = value.get("continue_tracking_p", {})
        if not isinstance(continuing, dict) or any(
            k not in ("1", "2", "3", "4", "5") for k in continuing
        ):
            raise ValueError("invalid continuation tracking map")
        continuing = tuple((int(k), integer(v, 3, 80)) for k, v in continuing.items())
        if any(sid in ids for sid, _ in continuing):
            raise ValueError("continuation is only for an axis without a new phase frame")
        return cls(
            value["name"],
            integer(value["duration_ms"], 1000 if ids == [5] else 1500, 9000),
            integer(value["dwell_ms"], 0, 500),
            tuple(targets),
            holds,
            continuing,
        )

    @property
    def commands(self):
        # 一个目标轴对应一条完整ASCII定位帧，本段各轴使用相同的声明T，非硬实时同步。
        return tuple(make_move_command(s, p, self.duration_ms) for s, p, _ in self.targets)


@dataclass(frozen=True)
class CycleSpec:
    # 两套六轴区间分别约束“允许开始的起点”和“整段允许观察到的P”，都不是厂家机械极限。
    review_id: str
    evidence: str
    start_intervals_p: tuple
    observation_bounds_p: tuple
    phases: tuple
    deadline_ms: int
    geometry: Geometry
    # 初始目标历史来自显式计划审阅，不是把位置回读误当成舵机内部当前目标寄存器。
    initial_setpoints_p: tuple = ()

    @classmethod
    def from_dict(cls, value):
        required = {
            "schema_version",
            "review_id",
            "whole_path_reviewed_for_motion",
            "evidence",
            "start_intervals_p",
            "observation_bounds_p",
            "phases",
            "deadline_ms",
            "geometry_mm",
        }
        if (
            not isinstance(value, dict)
            or required - set(value)
            or set(value) - required - {"initial_setpoints_p"}
        ):
            raise ValueError("provide complete reviewed sequence schema")
        if type(value["schema_version"]) is not int or value["schema_version"] != 1:
            raise ValueError("unsupported schema")
        if value["whole_path_reviewed_for_motion"] is not True:
            raise ValueError("whole sequence must be reviewed")
        if not isinstance(value["review_id"], str) or not re.fullmatch(
            r"[A-Za-z0-9_-]{1,80}", value["review_id"]
        ):
            raise ValueError("invalid bounded review id")
        if not isinstance(value["evidence"], str) or not value["evidence"].strip():
            raise ValueError("review evidence required")
        # 区间列表固定按000..005索引；后面可以用bounds[种类][servo_id]直接查该轴界限。
        bounds = []
        for key in ("start_intervals_p", "observation_bounds_p"):
            rows = value[key]
            if not isinstance(rows, list) or len(rows) != 6:
                raise ValueError("six explicit axis intervals required")
            checked = []
            for row in rows:
                if not isinstance(row, list) or len(row) != 2:
                    raise ValueError("invalid interval")
                low, high = [integer(v, 500, 2500) for v in row]
                if low > high:
                    raise ValueError("reversed interval")
                checked.append((low, high))
            bounds.append(tuple(checked))
        if any(not lo <= a <= b <= hi for (a, b), (lo, hi) in zip(*bounds)):
            raise ValueError("start outside observation bounds")
        if not isinstance(value["phases"], list):
            raise ValueError("phases must be a list")
        phases = tuple(Phase.from_dict(p) for p in value["phases"])
        if not 1 <= len(phases) <= 12 or len({p.name for p in phases}) != len(phases):
            raise ValueError("one..twelve uniquely named phases required")
        # 提前限制整趟最大定位帧数和阶段数，预算不能在运行中无限追加。
        if sum(len(p.targets) for p in phases) > 24:
            raise ValueError("sequence exceeds finite24-frame budget")
        deadline = integer(value["deadline_ms"], 4000, 20000)
        # 声明T与dwell不能占满总期限，还要给反馈查询和主机调度留下时间。
        if sum(p.duration_ms + p.dwell_ms for p in phases) > deadline - 1000:
            raise ValueError("reserve at least1s for feedback within the deadline")
        initial = value.get("initial_setpoints_p", {})
        if not isinstance(initial, dict) or any(k not in ("1", "2", "3", "5") for k in initial):
            raise ValueError("initial history only for explicitly reviewed body/gripper goals")
        initial = tuple((int(k), integer(v, 500, 2500)) for k, v in initial.items())
        if any(not bounds[1][sid][0] <= target <= bounds[1][sid][1] for sid, target in initial):
            raise ValueError("initial goal outside reviewed envelope")
        # 顺序推演每阶段已经拥有的目标，拒绝把“继续旧目标”伪装成未经预算的新动作。
        last_targets = dict(initial)
        for phase in phases:
            if any(sid not in last_targets for sid, _ in phase.continue_tracking_p):
                raise ValueError(
                    "continuation requires an earlier reserved goal or reviewed initial history"
                )
            for sid, target, _ in phase.targets:
                if not bounds[1][sid][0] <= target <= bounds[1][sid][1]:
                    raise ValueError("phase target outside reviewed axis bound")
                if sid in dict(phase.hold_tracking_p) and last_targets.get(sid) != target:
                    raise ValueError(
                        "hold cannot disguise a new target; same reserved prior setpoint required"
                    )
                last_targets[sid] = target
        return cls(
            value["review_id"],
            value["evidence"],
            *bounds,
            phases,
            deadline,
            Geometry(**value["geometry_mm"]),
            initial,
        )


class _SequenceSerial(SerialTransport):
    def write(self, command):
        # 底层仅写字节；本序列使用下面CycleTransport的精确帧门禁，而非旧局部步长门禁。
        return self.port.write(command)


class CycleTransport:
    """Exact current phase only, with one persistent reservation for the task."""

    def __init__(self, port, spec, *, raw=None):
        if not isinstance(spec, CycleSpec):
            raise ValueError("validated sequence required")
        self.spec = spec
        self.raw = _SequenceSerial(port) if raw is None else raw
        self.source = self.raw.source
        self.armed = self.cancelled = False
        # 两个游标分别指向当前阶段及阶段内下一条允许的帧，初值-1表示尚未进入阶段。
        self.phase_index, self.command_index = -1, 0
        # attempted_ids记录尝试写出的轴；stop_used保证异常清理每轴至多尝试一次PDST。
        self.attempted_ids, self.stop_used = set(), set()

    def open(self):
        self.raw.open()

    def close(self):
        self.raw.close()

    @property
    def closed(self):
        return self.raw.closed

    @property
    def in_waiting(self):
        return self.raw.in_waiting

    def read(self, count):
        return self.raw.read(count)

    def arm_after_reservation(self):
        # 只有持久化整趟预算后才允许调用一次；取消后不能重新arm继续用旧预算。
        if self.armed or self.cancelled:
            raise RuntimeError("sequence reservation already consumed")
        self.armed = True

    def begin_phase(self, index):
        # 只能进入紧接着的下一阶段，而且上一阶段的定位帧必须已按顺序消费完。
        if not self.armed or self.cancelled or index != self.phase_index + 1:
            raise RuntimeError("phase cannot be skipped, replayed or resumed")
        if self.phase_index >= 0 and self.command_index != len(
            self.spec.phases[self.phase_index].commands
        ):
            raise RuntimeError("previous phase incomplete")
        if not 0 <= index < len(self.spec.phases):
            raise ValueError("phase outside reserved sequence")
        self.phase_index, self.command_index = index, 0

    def write(self, command):
        if type(command) is not bytes:
            raise ValueError("exact bytes required")
        try:
            # 白名单查询不消耗运动帧；运动和停止请求进入下面更严格的分支。
            validate_read_query(command)
        except ValueError:
            sid = next(
                (s for s in self.attempted_ids if command == f"#{s:03d}PDST!".encode()), None
            )
            if sid is not None:
                # 停止预算先消费再写，写失败也不会自动重复；这只是尽力请求。
                if sid in self.stop_used:
                    raise RuntimeError("stop allowance already consumed")
                self.stop_used.add(sid)
                self.cancelled = True
            else:
                if not self.armed or self.cancelled or self.phase_index < 0:
                    raise RuntimeError("no reserved active phase")
                phase = self.spec.phases[self.phase_index]
                if (
                    self.command_index >= len(phase.commands)
                    or command != phase.commands[self.command_index]
                ):
                    raise ValueError("not the next exact phase frame")
                # 写入之前登记“可能已执行”：即使底层短写/报错，也不能当作完全没发过。
                self.attempted_ids.add(phase.targets[self.command_index][0])
                self.command_index += 1  # consumed even on a partial/failed write
        return self.raw.write(command)


class CycleController(JointProbeController):
    def __init__(self, link, spec, reserve, *, cancel_file=None):
        if (
            not isinstance(link.transport, CycleTransport)
            or link.transport.spec != spec
            or not callable(reserve)
        ):
            raise ValueError("matching sequence gate and reservation callback required")
        self.link, self.spec, self.reserve = link, spec, reserve
        self.cancel_file = Path(cancel_file) if cancel_file else None
        self.started = None
        self.used = self.cancel_requested = False

    def _check_cancel(self):
        # cancel_file由外部请求取消；使用父类检查同一控制流程的取消/异常状态。
        if self.cancel_file and self.cancel_file.exists():
            self.cancel_requested = True
        super()._check_cancel()
        # monotonic是只向前计时的主机秒数，避免系统日期调整影响动作总期限。
        if (
            self.started is not None
            and self.link.clock.monotonic() - self.started > self.spec.deadline_ms / 1000
        ):
            raise TimeoutError("whole-cycle20s-or-reviewed deadline exceeded")

    def guard(self, positions, starts, moving, holds=None):
        # positions/starts按舵机ID索引；moving是本段新位移目标，holds是已拥有目标的走廊。
        self._check_cancel()
        holds = holds or {}
        for sid, p in enumerate(positions):
            low, high = self.spec.observation_bounds_p[sid]
            if type(p) is not int or not low <= p <= high:
                raise ValueError("feedback outside reviewed sequence envelope")
            if sid in moving:
                # 明确新位移时检查方向：超过2P的反向变化触发失败，不把反向当普通残差。
                target = moving[sid]
                if (
                    abs(target - starts[sid]) > 3
                    and (p - starts[sid]) * (1 if target > starts[sid] else -1) < -2
                ):
                    raise ValueError("active axis moved opposite to requested direction")
            elif sid in holds:
                # 持续跟踪以原先下发的目标为基准，不以每次回读重新设置宽松目标。
                target, tolerance = holds[sid]
                if abs(p - target) > tolerance:
                    raise ValueError("ordered unchanged setpoint exceeded tracking corridor")
            elif abs(p - starts[sid]) > 10:
                raise ValueError("held axis drift exceeded10P")
        self.guard_geometry(positions)

    def guard_geometry(self, positions):
        # 默认保留原序列的几何规则；通用入口任务可使用与规划一致的场景检查器。
        # 仅000..003参与四轴名义几何；005夹爪反馈不代表已经夹住物体。
        points = chain_points(self.spec.geometry, positions[:4])
        q = from_nominal_pulses(positions[:4])
        if not (
            -90 <= q.yaw_deg <= 90
            and 0 <= q.shoulder_deg <= 180
            and -135 <= q.elbow_flex_deg <= 135
            and -90 <= q.wrist_deg <= 90
        ):
            raise ValueError("feedback outside nominal model angular bounds")
        yaw = math.radians(-(positions[0] - 1500) * 0.135)
        # 这里检查名义中心线桌面余量和已清空工作区，不能作为实测碰撞/毫米精度证明。
        if min(point[2] for point in points) < 5:
            raise ValueError("nominal chain below5mm tabletop margin")
        if max(math.hypot(p[0], p[1]) for p in points) + 35 > 400:
            raise ValueError("nominal chain plus object leaves cleared40cm")
        if min(
            p[0] * math.sin(yaw) + p[1] * math.cos(yaw) for p in points
        ) < -5 and not is_nominal_upright_reference(
            self.spec.geometry, positions[:4], points=points
        ):
            raise ValueError("nominal chain folds behind forward workspace")

    @staticmethod
    def positions(state):
        if state["status"] != "OK":
            raise ValueError("incomplete six-axis feedback")
        return [s["position_p"] for s in state["servos"]]

    def stop_attempted(self, result):
        # 异常时仅向已尝试运动的轴各请求一次停止；不释放扭矩、不回零、不补发原动作。
        for sid in sorted(self.link.transport.attempted_ids):
            if sid in self.link.transport.stop_used:
                continue
            item = {"servo_id": sid, "physical_stop_verified": False}
            try:
                command = f"#{sid:03d}PDST!".encode()
                item["bytes_written"] = self.link.transport.write(command)
                if item["bytes_written"] != len(command):
                    item["error"] = "short stop write"
            except Exception as error:
                item["error"] = type(error).__name__ + ": " + str(error)
            result["stops"].append(item)
            self.link.record_best_effort("cycle_stop_attempt", **item)
        self.link.faulted = True

    def check_completion(self):
        """Allow task-specific completion conditions before success is recorded."""
        pass

    def run(self):
        # 一个控制器只消费一趟序列，已故障或日志不完整时不能继续靠旧状态运行。
        if self.used or self.link.faulted or self.link.log_errors:
            raise RuntimeError("sequence already used, faulted or unlogged")
        self.used = True
        result = {
            "source": self.link.source,
            "status": "PREFLIGHT",
            "phases": [],
            "error": None,
            "response_pass": False,
            "physical_task_verified": False,
            "physical_stop_verified": False,
            "strict_position_verified": False,
            "stops": [],
        }
        reader = ArmStateReader(self.link)
        try:
            self._check_cancel()
            identities = {}
            for sid in sorted(
                {t[0] for phase in self.spec.phases for t in phase.targets}
                | set(dict(self.spec.initial_setpoints_p))
            ):
                identities[sid] = {"version": self.link.query(sid, "version")}
                self._wait(0.05)
                identities[sid]["mode"] = self.link.query(sid, "mode")
                if (
                    identities[sid]["version"]["value"] != EXPECTED_FIRMWARE
                    or identities[sid]["mode"]["value"] != 1
                ):
                    raise ValueError("unexpected servo firmware or mode")
            result["identity"] = identities
            # 三次新鲜六轴读取核对起点稳定性；六轴仍是顺序读取，不是硬件同步快照。
            states = []
            for _ in range(3):
                states.append(reader.read_state())
                self._wait(0.05)
            rows = [self.positions(state) for state in states]
            for sid, (low, high) in enumerate(self.spec.start_intervals_p):
                values = [row[sid] for row in rows]
                if max(values) - min(values) > 2 or any(not low <= v <= high for v in values):
                    raise ValueError("fresh stable starting pose differs from review")
            result["initial_state"] = states[-1]
            current = rows[-1]
            self.guard(current, current, {})
            # reserve由调用方落盘整趟精确帧预算；持久化成功后才允许arm和第一次运动。
            self.reserve(
                {
                    "start6_p": current,
                    "commands": [[c.decode() for c in p.commands] for p in self.spec.phases],
                }
            )
            self._check_cancel()
            self.link.transport.arm_after_reservation()
            # 正式循环计时从预算成功后的执行开始；身份核验和起点读取属于预检。
            self.started = self.link.clock.monotonic()
            result["started_at"] = self.link.clock.timestamp()
            # 保存本趟已经下发/已审阅的目标归属，跨phase监测同一目标时不偷偷改成新回读值。
            ordered_targets = dict(self.spec.initial_setpoints_p)
            tracking_limits = {}
            for index, phase in enumerate(self.spec.phases):
                self._check_cancel()
                # 每阶段保存本段真实起点副本；moving用于方向检查，holds用于旧目标持续跟踪。
                starts = list(current)
                hold_bounds = dict(phase.hold_tracking_p)
                moving = {s: p for s, p, _ in phase.targets if s not in hold_bounds}
                holds = {s: (p, hold_bounds[s]) for s, p, _ in phase.targets if s in hold_bounds}
                for sid, tolerance in phase.continue_tracking_p:
                    # 这里只添加监测条件，后面的phase.commands不会包含这些轴的新定位帧。
                    holds[sid] = (ordered_targets[sid], tolerance)
                    tracking_limits[sid] = tolerance
                row = {
                    "name": phase.name,
                    "starts_p": starts,
                    "samples": [],
                    "end_active": [],
                    "response_pass": False,
                }
                result["phases"].append(row)
                self.guard(current, starts, moving, holds)
                self.link.transport.begin_phase(index)
                # 本段只按精确清单连续发送，不在每个几何抽样点重新定位或强制停顿。
                phase_start = self.link.clock.monotonic()
                for command in phase.commands:
                    self._check_cancel()
                    self.link.send_motion(command)
                # 每条轴目标的T从实际发送后才开始生效；日志、调度和前面轴的发送
                # 都可能耗时。以最后一条目标写完为保守等待基准，防止尚未给足T
                # 就读取端点并误停；phase_start仍保留整段实际耗时，总期限不后移。
                motion_wait_start = self.link.clock.monotonic()
                for sid, target, tolerance in phase.targets:
                    ordered_targets[sid] = target
                    tracking_limits[sid] = tolerance
                row["commands_sent"] = [c.decode() for c in phase.commands]
                row["continuing_goals"] = {
                    str(s): {"target_p": p, "tracking_tolerance_p": t}
                    for s, (p, t) in holds.items()
                    if s not in moving
                }
                self.link.record(
                    "cycle_phase_started", name=phase.name, elapsed_s=phase_start - self.started
                )
                # Reserve the last0.4s for a complete all-axis query, then check
                # active axes after T. No additional diagnostic2s settling pause.
                due = 0.5
                duration = phase.duration_ms / 1000
                # due是相对本段起点的秒数；按绝对截止时刻等待，减少每轮sleep叠加漂移。
                while due <= duration - 0.4 + 1e-9:
                    self._wait(max(0, motion_wait_start + due - self.link.clock.monotonic()))
                    state = reader.read_state()
                    self._check_cancel()
                    current = self.positions(state)
                    self.guard(current, starts, moving, holds)
                    row["samples"].append(state)
                    self.link.record("cycle_sample", phase=phase.name, state=state)
                    due += 0.5
                    # 慢查询跨过多个采样点时跳过已过期的时刻，避免追赶旧采样
                    # 延误阶段交接；仍通过guard/_wait和每次真实write核对总期限。
                    elapsed = self.link.clock.monotonic() - motion_wait_start
                    due = max(due, (math.floor((elapsed + 1e-9) / 0.5) + 1) * 0.5)
                # A reviewed dwell belongs to this motion phase. Read active
                # endpoints AFTER it, so the next phase's held-axis baseline
                # includes the prior command's settling instead of stale P.
                self._wait(
                    max(
                        0,
                        motion_wait_start
                        + duration
                        + phase.dwell_ms / 1000
                        - self.link.clock.monotonic(),
                    )
                )
                # 等声明T和计划dwell结束后再读活动轴，形成下一阶段可用的较新交接反馈。
                for sid, target, tolerance in phase.targets:
                    reply = self.link.query(sid)
                    current[sid] = reply["value"]
                    self._check_cancel()
                    low, high = self.spec.observation_bounds_p[sid]
                    integer(current[sid], low, high)
                    change = current[sid] - starts[sid]
                    # reached表示处于审阅的交接容差；strict_within3p另记，二者不等价。
                    reached = abs(target - current[sid]) <= tolerance
                    directed = change * (target - starts[sid]) > 0 and abs(change) > 2
                    near = abs(target - starts[sid]) <= 3 and abs(target - current[sid]) <= 3
                    row["end_active"].append(
                        {
                            "servo_id": sid,
                            "target_p": target,
                            "feedback_p": current[sid],
                            "error_p": target - current[sid],
                            "handoff_tolerance_p": tolerance,
                            "in_handoff_corridor": reached,
                            "direction_response": directed,
                            "already_near_target": near,
                            "observed_delta_p": change,
                            "unchanged_ordered_target_hold": sid in holds,
                            "strict_within3p": abs(target - current[sid]) <= 3,
                        }
                    )
                    if not reached or not (directed or near or sid in holds):
                        raise ValueError(
                            "phase lacks directed response within reviewed handoff corridor"
                        )
                # Both active-axis end reads are collected before a geometric
                # check. Mixing one new axis with the other axis's0.4s-old
                # sample can invent a posture the hardware never occupied.
                # 活动轴端点都读完后再组合检查，减轻新旧轴混时；仍不声称是同步姿态。
                self.guard(current, starts, moving, holds)
                row["response_pass"] = True
                self.link.record("cycle_phase_end", phase=phase.name, end_active=row["end_active"])
            # 结束前再读完整六轴，并按整趟最后拥有的目标核对，不能只凭最后一阶段成功返回。
            result["final_state"] = reader.read_state()
            final = self.positions(result["final_state"])
            final_holds = {
                sid: (target, tracking_limits[sid])
                for sid, target in ordered_targets.items()
                if sid in tracking_limits
            }
            self.guard(final, current, {}, final_holds)
            result["final_commanded_targets_p"] = {str(s): p for s, p in ordered_targets.items()}
            result["final_tracking_errors_p"] = {
                str(s): final[s] - p for s, p in ordered_targets.items()
            }
            self.check_completion()
            result.update(status="RESPONSE_SEQUENCE_COMPLETED", response_pass=True)
        except (Exception, KeyboardInterrupt) as error:
            # 是否尝试过运动决定失败标签；任何错误都不会从当前阶段自动重试。
            result.update(
                status="FAILED_AFTER_ATTEMPT"
                if self.link.transport.attempted_ids
                else "PREFLIGHT_FAILED",
                error=type(error).__name__ + ": " + str(error),
            )
        finally:
            # 最终读取/日志也会消耗时间，所以在finally再核总期限，防止边界竞态误报成功。
            if self.started is not None:
                result["elapsed_s"] = self.link.clock.monotonic() - self.started
                result["within_deadline"] = result["elapsed_s"] <= self.spec.deadline_ms / 1000
                if not result["within_deadline"] and not result["error"]:
                    result.update(
                        status="FAILED_AFTER_ATTEMPT",
                        response_pass=False,
                        error="TimeoutError: final completion exceeded reviewed cycle deadline",
                    )
            # 日志不完整同样使验收失败；停止请求及其物理效果分别记录，不能把关闭视为急停。
            if result["error"] or self.link.log_errors:
                result["response_pass"] = False
                self.stop_attempted(result)
            self.link.record_best_effort("cycle_result", result=result)
            if self.link.log_errors:
                result.update(
                    status="LOGGING_FAILED", error="cycle logging incomplete", response_pass=False
                )
                self.stop_attempted(result)
        return result
