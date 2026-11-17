"""到货后手动运行的 Wi-Fi 单项检查；不是自动启动 main.py。"""

from wifi_link import connect_wifi


def main():
    try:
        from wifi_config import WIFI_SSID, WIFI_PASSWORD
    except ImportError:
        print("Create local wifi_config.py using wifi_config.example.py first.")
        return
    if WIFI_SSID == "YOUR_2_4G_WIFI_NAME" or WIFI_PASSWORD == "YOUR_WIFI_PASSWORD":
        print("Replace the example Wi-Fi settings locally before running.")
        return
    print("ESP32 Wi-Fi setup only; no servo commands.")
    wlan = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    print("Wi-Fi connected. IP:", wlan.ifconfig()[0])
    print("TCP service is not started in this lesson.")


if __name__ == "__main__":
    main()
