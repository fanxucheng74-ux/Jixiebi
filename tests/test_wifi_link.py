import unittest

from runpy import run_path
from arm_project.paths import PROJECT_ROOT
from arm_project.cli.wifi_pc_demo import FakeClock, FakeNetwork

connect_wifi = run_path(str(PROJECT_ROOT / "firmware/esp32/wifi_link.py"))["connect_wifi"]


class WifiLogicTests(unittest.TestCase):
    def setup_case(self, scenario, clock=None):
        clock = clock or FakeClock()
        return clock, FakeNetwork(clock, scenario)

    def connect(self, clock, network):
        return connect_wifi(
            "DEMO_ONLY", "NOT_A_REAL_PASSWORD", timeout_ms=1000, network_module=network, clock=clock
        )

    def test_delayed_success_retains_connection(self):
        clock, network = self.setup_case("success")
        wlan = self.connect(clock, network)
        self.assertTrue(wlan.isconnected())
        self.assertEqual(clock.elapsed, 300)
        self.assertEqual(wlan.active_history, [False, True])

    def test_connecting_forever_times_out_and_disables_sta(self):
        clock, network = self.setup_case("timeout")
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.connect(clock, network)
        self.assertEqual(clock.elapsed, 1000)
        self.assertFalse(network.wlan.enabled)

    def test_reported_password_error_disables_sta(self):
        clock, network = self.setup_case("password_error")
        with self.assertRaisesRegex(RuntimeError, "STAT_WRONG_PASSWORD"):
            self.connect(clock, network)
        self.assertFalse(network.wlan.enabled)

    def test_timeout_across_clock_wrap(self):
        clock, network = self.setup_case("timeout", FakeClock(FakeClock.PERIOD - 150))
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.connect(clock, network)
        self.assertEqual(clock.elapsed, 1000)
        self.assertFalse(network.wlan.enabled)

    def test_interrupted_wait_disables_sta(self):
        class InterruptedClock(FakeClock):
            def sleep_ms(self, milliseconds):
                raise KeyboardInterrupt

        clock, network = self.setup_case("timeout", InterruptedClock())
        with self.assertRaises(KeyboardInterrupt):
            self.connect(clock, network)
        self.assertFalse(network.wlan.enabled)

    def test_previous_connection_not_mistaken_for_new_success(self):
        clock, network = self.setup_case("success")
        network.wlan.enabled = True
        network.wlan.started = -500
        self.assertTrue(network.wlan.isconnected())
        self.connect(clock, network)
        self.assertEqual(clock.elapsed, 300)
        self.assertEqual(network.wlan.active_history, [False, True])


if __name__ == "__main__":
    unittest.main()
