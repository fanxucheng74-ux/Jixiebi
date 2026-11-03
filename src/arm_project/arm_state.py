"""与ROS无关的六舵机位置接口；使用协议P值，不假定角度或同步采样。"""

from threading import Lock


class ArmStateReader:
    def __init__(self, link):
        # link指向同一串口事务对象；锁阻止同一reader的两次读取交叉消耗回复。
        self.link = link
        self._lock = Lock()

    def read_identity(self):
        """连接时读取版本、ID和模式；后续read_state不重复这组查询。"""
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("another arm read is running")
        identities = []
        try:
            # 000–005逐个读取，replies保留每类回复和主机接收时间，便于追溯固件/模式。
            for servo_id in range(6):
                item = {"servo_id": servo_id, "replies": {}}
                for field in ("version", "id", "mode"):
                    item["replies"][field] = self.link.query(servo_id, field)
                    self.link.clock.sleep(0.05)
                identities.append(item)
            self.link.record("arm_identity", identities=identities)
            return identities
        finally:
            self._lock.release()

    def read_state(self):
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("read_state is already running")
        try:
            return self._read_state()
        finally:
            self._lock.release()

    def _read_state(self):
        clock = self.link.clock
        started = clock.monotonic()
        # 六轴是顺序查询的快照，不是硬件同步采样；P值也不能直接当角度或毫米。
        state = {
            "source": self.link.source,
            "status": "OK",
            "started_at": clock.timestamp(),
            "position_unit": "protocol_P",
            "simultaneous": False,
            "timestamp_origin": "host_receive",
            "servos": [
                # 先为全部地址保留位置；读到一半失败时，未读轴仍明确为NOT_READ/None。
                {
                    "servo_id": i,
                    "status": "NOT_READ",
                    "position_p": None,
                    "received_at": None,
                    "received_monotonic_s": None,
                    "age_ms_at_finish": None,
                }
                for i in range(6)
            ],
            "error": None,
        }
        for servo_id in range(6):
            item = state["servos"][servo_id]
            try:
                reply = self.link.query(servo_id, "position")
            except (OSError, RuntimeError, ValueError) as error:
                # 保留已经读到的轴并标整个快照不完整，不用上次历史值填补本次缺失。
                item["status"] = "READ_FAILED"
                state["status"] = "INCOMPLETE"
                state["error"] = "%s: %s" % (type(error).__name__, error)
                break
            item.update(
                status="OK",
                position_p=reply["value"],
                received_at=reply["received_at"],
                received_monotonic_s=reply["received_monotonic_s"],
            )
            if servo_id < 5:
                clock.sleep(0.05)
        finished = clock.monotonic()
        state["finished_at"] = clock.timestamp()
        state["finished_monotonic_s"] = finished
        state["duration_ms"] = round((finished - started) * 1000, 3)
        for item in state["servos"]:
            if item["received_monotonic_s"] is not None:
                # age_ms_at_finish为该回复距整次读取结束经过的时间；越先读的轴通常越旧。
                item["age_ms_at_finish"] = round(
                    (finished - item["received_monotonic_s"]) * 1000, 3
                )
        self.link.record("arm_state", state=state)
        return state
