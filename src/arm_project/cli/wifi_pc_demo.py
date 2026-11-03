"""电脑模拟 Wi-Fi 连接状态，用同一个连接函数验证流程，没有无线收发。"""

import argparse

from runpy import run_path
from arm_project.paths import PROJECT_ROOT

connect_wifi = run_path(str(PROJECT_ROOT / "firmware/esp32/wifi_link.py"))["connect_wifi"]


class FakeClock:
    PERIOD = 1 << 20

    def __init__(self, start=0):
        self.start = start
        self.elapsed = 0

    def ticks_ms(self):
        return (self.start + self.elapsed) % self.PERIOD

    def ticks_diff(self, new, old):
        half = self.PERIOD // 2
        return (new - old + half) % self.PERIOD - half

    def sleep_ms(self, milliseconds):
        # 推进模拟时钟，不让电脑真的等待。
        self.elapsed += milliseconds


class FakeWLAN:
    def __init__(self, clock, scenario):
        self.clock = clock
        self.scenario = scenario
        self.enabled = False
        self.started = None
        self.active_history = []

    def active(self, enabled):
        self.enabled = enabled
        self.active_history.append(enabled)
        if not enabled:
            self.started = None

    def connect(self, ssid, password):
        self.started = self.clock.elapsed

    def isconnected(self):
        return (
            self.enabled
            and self.started is not None
            and self.scenario == "success"
            and self.clock.elapsed - self.started >= 300
        )

    def status(self):
        if self.isconnected():
            return 3
        if self.scenario == "password_error" and self.clock.elapsed - self.started >= 200:
            return -3
        return 1

    def ifconfig(self):
        if not self.isconnected():
            raise RuntimeError("fake device has no IP")
        # 文档示例地址，不是用户网络中可连接的设备地址。
        return ("192.0.2.32", "255.255.255.0", "192.0.2.1", "192.0.2.1")


class FakeNetwork:
    STA_IF = 0
    STAT_WRONG_PASSWORD = -3
    STAT_NO_AP_FOUND = -2
    STAT_CONNECT_FAIL = -1

    def __init__(self, clock, scenario):
        self.wlan = FakeWLAN(clock, scenario)

    def WLAN(self, interface):
        if interface != self.STA_IF:
            raise ValueError("expected STA interface")
        return self.wlan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario", choices=("success", "password_error", "timeout"), default="success"
    )
    args = parser.parse_args()
    clock = FakeClock()
    network = FakeNetwork(clock, args.scenario)
    print("SIMULATION ONLY: no Wi-Fi radio or real network connection.")
    print("STA init -> activate -> connect -> wait for IP")
    try:
        wlan = connect_wifi(
            "DEMO_ONLY", "NOT_A_REAL_PASSWORD", timeout_ms=1000, network_module=network, clock=clock
        )
    except RuntimeError as error:
        print("SIMULATED ERROR:", error)
        print("STA disabled:", not network.wlan.enabled)
        return 1
    print("SIMULATED IP:", wlan.ifconfig()[0], "(documentation example, not a real device)")
    print("SIMULATED elapsed:", clock.elapsed, "ms")
    print("PASS: connection logic only; ESP32 hardware remains untested.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
