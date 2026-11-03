"""ROS 只读节点共用的串口会话；本模块不依赖 ROS，也不在导入时打开硬件。"""

from datetime import datetime
import json
from uuid import uuid4

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_state import ArmStateReader
from arm_project.servo_protocol import validate_read_query


class ReadOnlySerialTransport(SerialTransport):
    """在串口写入入口拒绝所有非查询帧，包括停止帧和运动帧。"""

    def write(self, command):
        validate_read_query(command)
        return super().write(command)


class ReadOnlySession:
    """一个会话独占一个串口；出错后关闭，不自动重连、不恢复旧状态。"""

    def __init__(self, port, runs_dir):
        if not port:
            raise ValueError("必须显式指定串口")
        self.transport = ReadOnlySerialTransport(port)
        # 每次新建独立会话目录，保留原始 TX/RX 和错误，不覆盖旧记录。
        self.session_id = uuid4().hex
        self.run_dir = runs_dir / ("ros_readonly_" + self.session_id)
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.log = (self.run_dir / "events.jsonl").open("x", encoding="utf-8")
        self.link = ServoLink(self.transport, record=self._record)
        self.reader = None
        self.entered = False
        self.closed = False
        self.batch_index = 0

    def _record(self, event, **values):
        self.log.write(
            json.dumps(
                {"time": datetime.now().astimezone().isoformat(), "event": event, **values},
                ensure_ascii=False,
            )
            + "\n"
        )
        self.log.flush()

    def start(self):
        if self.entered or self.closed:
            raise RuntimeError("会话不可重复启动")
        try:
            self.link.__enter__()
            self.entered = True
            self.reader = ArmStateReader(self.link)
            return self.reader.read_identity()
        except BaseException as error:
            self._close_after_error("startup", error)
            raise

    def poll(self):
        if not self.entered or self.closed:
            raise RuntimeError("会话未运行或已关闭")
        try:
            state = self.reader.read_state()
            self.batch_index += 1
            # 一旦有未读/失败的轴，发布该次不完整结果后停止，绝不拼接旧值。
            if state["status"] != "OK":
                self.close()
            return state
        except BaseException as error:
            self._close_after_error("poll", error)
            raise

    def _close_after_error(self, phase, error):
        # 故障日志与关闭均尽力执行；资源清理出错不覆盖原始通信错误。
        self.link.record_best_effort("session_error", phase=phase, error=str(error))
        try:
            self.close()
        except BaseException:
            pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.entered:
                self.link.__exit__(None, None, None)
        finally:
            self.log.close()
