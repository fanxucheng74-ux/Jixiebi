"""六轴只读状态到四轴标称模型；只做计算，不访问 ROS、串口或控制接口。"""

from datetime import datetime, timezone
import math
from uuid import UUID

from arm_project.kinematics import from_nominal_pulses


def _finite_number(value, name):
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _boot_id(value):
    # 此标识用于比较Linux同一启动周期，并不是消息源认证或时间命名空间证明。
    if not isinstance(value, str):
        raise ValueError("boot_id must be a UUID string")
    try:
        return str(UUID(value))
    except (ValueError, AttributeError) as error:
        raise ValueError("invalid boot_id") from error


def _timestamp(value):
    # received_at是主机收到该轴回复的墙钟时刻，不是舵机自身的采样时间。
    if not isinstance(value, str):
        raise ValueError("received_at must be a timestamp")
    try:
        instant = datetime.fromisoformat(value)
        if instant.tzinfo is None or instant.utcoffset() is None:
            raise ValueError("received_at must include timezone")
        utc = instant.astimezone(timezone.utc)
        delta = utc - datetime(1970, 1, 1, tzinfo=timezone.utc)
        seconds = delta.days * 86400 + delta.seconds
        if not -(2**31) <= seconds < 2**31:
            raise ValueError("received_at outside ROS Time range")
        return seconds, delta.microseconds * 1000
    except (ValueError, OverflowError) as error:
        raise ValueError("invalid received_at") from error


class NominalStateBridge:
    """Fail-closed模型转换器；固定首个有效session，切换需人工重启桥。"""

    def __init__(self, host_boot_id, max_age_s=3.0):
        self.host_boot_id = _boot_id(host_boot_id)
        self.max_age_s = _finite_number(max_age_s, "max_age_s")
        if not 0 < self.max_age_s <= 60:
            raise ValueError("max_age_s must be >0 and <=60")
        self._session_id = None
        self._last_batch = None
        self._last_monotonic = None

    def accept(self, payload, *, now_monotonic, now_wall, host_boot_id):
        """返回四轴独立标称JointState数据；任何异常均不推进会话或序号。"""
        now = _finite_number(now_monotonic, "now_monotonic")
        wall = _finite_number(now_wall, "now_wall")
        if now < 0 or _boot_id(host_boot_id) != self.host_boot_id:
            raise ValueError("local boot identity changed")
        if not isinstance(payload, dict) or payload.get("schema") != "jixiebi.raw_state.v1":
            raise ValueError("unknown raw state schema")
        session = payload.get("session_id")
        if not isinstance(session, str) or not session or len(session) > 128:
            raise ValueError("missing session_id")
        if self._session_id is not None and session != self._session_id:
            raise ValueError("session changed; restart bridge after inspection")
        batch = payload.get("batch_index")
        if type(batch) is not int or batch < 1:
            raise ValueError("invalid batch_index")
        if self._last_batch is not None and batch <= self._last_batch:
            raise ValueError("duplicate or out-of-order batch")
        state = payload.get("state")
        if not isinstance(state, dict):
            raise ValueError("state must be an object")
        if (
            state.get("source") != "HARDWARE"
            or state.get("status") != "OK"
            or "error" not in state
            or state.get("error") is not None
            or state.get("position_unit") != "protocol_P"
            or state.get("simultaneous") is not False
            or state.get("timestamp_origin") != "host_receive"
            or _boot_id(state.get("host_boot_id")) != self.host_boot_id
        ):
            raise ValueError("raw state invalid or from a different boot")
        servos = state.get("servos")
        if not isinstance(servos, list) or len(servos) != 6:
            raise ValueError("expected six servo records")
        pulses, stamps, times = [], [], []
        for servo_id, item in enumerate(servos):
            if not isinstance(item, dict):
                raise ValueError("servo record must be an object")
            if type(item.get("servo_id")) is not int or item["servo_id"] != servo_id:
                raise ValueError("servo IDs must be ordered 0 through 5")
            pulse = item.get("position_p")
            if item.get("status") != "OK" or type(pulse) is not int or not 500 <= pulse <= 2500:
                raise ValueError("invalid servo position")
            received = _finite_number(item.get("received_monotonic_s"), "received_monotonic_s")
            if received < 0 or received > now or now - received > self.max_age_s:
                raise ValueError("sample stale, in future, or invalid")
            if times and received <= times[-1]:
                raise ValueError("servo receive times not increasing")
            stamp = _timestamp(item.get("received_at"))
            # 两种主机时钟的年龄应大致一致；时钟回拨或伪造的墙钟值不得进入TF。
            wall_age = wall - (stamp[0] + stamp[1] / 1_000_000_000)
            if abs(wall_age - (now - received)) > 1.0:
                raise ValueError("wall and monotonic timestamps disagree")
            if stamps and stamp < stamps[-1]:
                raise ValueError("host receive timestamps not ordered")
            if self._last_monotonic is not None and received <= self._last_monotonic[servo_id]:
                raise ValueError("stale servo timestamp")
            pulses.append(pulse)
            stamps.append(stamp)
            times.append(received)

        # from_nominal_pulses只是厂家标称比例；URDF四轴符号还需按模型约定转换。
        nominal = from_nominal_pulses(pulses[:4])
        angles = (
            -nominal.yaw_deg,
            nominal.shoulder_deg,
            -nominal.elbow_flex_deg,
            nominal.wrist_deg,
        )
        names = ("base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch")
        result = [
            {
                "name": name,
                "position_rad": math.radians(angle),
                "stamp_sec": stamps[index][0],
                "stamp_nanosec": stamps[index][1],
            }
            for index, (name, angle) in enumerate(zip(names, angles))
        ]
        # 成功构造所有数据后才提交状态，损坏报文不能污染序号与会话。
        self._session_id = session
        self._last_batch = batch
        self._last_monotonic = tuple(times)
        return result
