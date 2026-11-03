"""ESP32 MicroPython 1.18：初始化 STA、连接路由器并等待 IP。

仅在显式调用时操作网络，不导入舵机程序。重新调用会重置 STA，
所以这是启动时的连接步骤，不是运行中的心跳检查函数。
network_module/clock 供电脑注入模拟设备；板端使用时无需传入。
"""


def connect_wifi(ssid, password, timeout_ms=15000, network_module=None, clock=None):
    if not isinstance(ssid, str) or not ssid:
        raise ValueError("Wi-Fi SSID must be a nonempty string")
    if not isinstance(password, str):
        raise ValueError("Wi-Fi password must be a string")
    if type(timeout_ms) is not int or not 1 <= timeout_ms <= 60000:
        raise ValueError("timeout_ms must be between 1 and 60000")
    if network_module is None:
        import network as network_module
    if clock is None:
        import time as clock

    wlan = network_module.WLAN(network_module.STA_IF)
    connected = False
    try:
        # 从关闭状态开始，防止把旧网络的连接状态误认为新连接成功。
        wlan.active(False)
        wlan.active(True)
        started = clock.ticks_ms()
        wlan.connect(ssid, password)

        while not wlan.isconnected():
            status = wlan.status()
            for name in ("STAT_WRONG_PASSWORD", "STAT_NO_AP_FOUND", "STAT_CONNECT_FAIL"):
                if hasattr(network_module, name) and status == getattr(network_module, name):
                    raise RuntimeError("Wi-Fi reported " + name)
            # 某些 ESP32 固件出错后仍报告 CONNECTING，因此还需独立超时。
            if clock.ticks_diff(clock.ticks_ms(), started) >= timeout_ms:
                raise RuntimeError("Wi-Fi connection timed out; no valid IP received")
            clock.sleep_ms(100)

        connected = True
        return wlan
    finally:
        if not connected:
            # 异常或 Ctrl+C 后关闭 STA，停止驱动在后台继续尝试连接。
            try:
                wlan.active(False)
            except Exception:
                pass  # 保留原来的连接异常，不用清理异常覆盖它。
