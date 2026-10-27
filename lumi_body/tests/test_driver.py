# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the body-joint driver against a fake vendor client.

Covers config validation, the mm/deg <-> m/rad boundary, the motion gate, the
limit checks and the stale-value policy. No robot, no HTTP.
"""
import math
import unittest

from lumi_body.driver import (
    JOINT_NAMES,
    LumiBodyDriver,
    api_to_ros,
    parse_config,
    ros_to_api,
)
from lumi_body.http_client import ExtAxisError

try:  # the publish path builds a real JointState
    import sensor_msgs.msg  # noqa: F401

    HAS_ROS = True
except Exception:  # noqa: BLE001
    HAS_ROS = False


def _axes(lift_mm=0.0, waist_deg=0.0, head_yaw_deg=0.0, head_pitch_deg=0.0,
          enable=1, error=0, ecode=0):
    positions = (lift_mm, waist_deg, head_yaw_deg, head_pitch_deg)
    return [
        {
            "id": index,
            "pos": value,
            "vel": 0.0,
            "enable": enable,
            "error": error,
            "ecode": ecode,
        }
        for index, value in enumerate(positions)
    ]


class FakeExtClient:
    def __init__(self, statuses=None):
        self.statuses = list(statuses or [_axes()])
        self.moveto_calls: list = []
        self.enable_calls: list = []
        self.stop_calls = 0
        self.fail = False

    def sysinfo(self):
        return {"serial_number": "Lumi10000023", "version": "1.0"}

    def status(self):
        if self.fail:
            raise ExtAxisError("boom")
        if len(self.statuses) > 1:
            return self.statuses.pop(0)
        return self.statuses[0]

    def enable(self, engaged):
        self.enable_calls.append(engaged)
        return {"status": 0}

    def moveto(self, positions, vel, acc):
        self.moveto_calls.append(([float(p) for p in positions], vel, acc))
        return {"status": 0}

    def stop(self):
        self.stop_calls += 1
        return {"status": 0}

    def reset(self):
        return {"status": 0}


def _driver(cfg=None, client=None, clock=None):
    normalized = parse_config(cfg if cfg is not None else {"motion_enabled": True})
    published = []
    drv = LumiBodyDriver(
        normalized,
        publish_joint_states=published.append,
        clock=clock,
        log=lambda text: None,
    )
    drv._client = client or FakeExtClient()
    return drv, published


class TestUnits(unittest.TestCase):
    def test_api_to_ros_converts_mm_and_deg(self):
        self.assertEqual(
            api_to_ros([100.0, 90.0, -180.0, 35.0]),
            (0.1, math.pi / 2, -math.pi, math.radians(35.0)),
        )

    def test_round_trip_is_stable(self):
        values = (0.25, math.radians(30.0), math.radians(-45.0), math.radians(10.0))
        self.assertEqual(api_to_ros(ros_to_api(values)), values)


class TestParseConfig(unittest.TestCase):
    def test_defaults(self):
        cfg = parse_config({})
        self.assertEqual(cfg["base_url"], "http://192.168.10.90:5000")
        self.assertFalse(cfg["motion_enabled"])
        self.assertTrue(cfg["auto_enable"])
        self.assertEqual(cfg["joint_states_topic"], "/joint_states")
        self.assertEqual(cfg["lift_max_mm"], 300.0)
        self.assertEqual(cfg["waist_max_deg"], 140.0)
        self.assertIsNone(cfg["shutdown_home"])

    def test_base_url_trailing_slash_is_removed(self):
        self.assertEqual(
            parse_config({"base_url": "http://192.168.10.90:5000/"})["base_url"],
            "http://192.168.10.90:5000",
        )

    def test_lift_above_urdf_travel_is_rejected(self):
        with self.assertRaises(ValueError) as ctx:
            parse_config({"lift_max_mm": 450.0})
        self.assertIn("400", str(ctx.exception))

    def test_waist_above_urdf_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_config({"waist_max_deg": 170.0})

    def test_full_scale_velocity_is_accepted(self):
        cfg = parse_config({"default_vel_pct": 100, "default_acc_pct": 100})
        self.assertEqual(cfg["default_vel_pct"], 100)
        self.assertEqual(cfg["default_acc_pct"], 100)

    def test_velocity_above_full_scale_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_config({"default_vel_pct": 101})

    def test_shutdown_home_needs_four_numbers(self):
        with self.assertRaises(ValueError):
            parse_config({"shutdown_home": [0.0, 0.0, 0.0]})
        self.assertEqual(
            parse_config({"shutdown_home": [1, 2, 3, 4]})["shutdown_home"],
            (1.0, 2.0, 3.0, 4.0),
        )

    def test_boolean_strings(self):
        self.assertTrue(parse_config({"motion_enabled": "yes"})["motion_enabled"])
        self.assertFalse(parse_config({"motion_enabled": "off"})["motion_enabled"])

    def test_bad_boolean(self):
        with self.assertRaises(ValueError):
            parse_config({"auto_enable": "perhaps"})


class TestLimits(unittest.TestCase):
    def test_default_limits_use_the_conservative_api_ranges(self):
        drv, _ = _driver()
        self.assertEqual(
            drv.limits_api(),
            ((0.0, 300.0), (-140.0, 140.0), (-180.0, 180.0), (-5.0, 35.0)),
        )

    def test_relaxed_limits_follow_the_config(self):
        drv, _ = _driver({"motion_enabled": True, "lift_max_mm": 400.0,
                          "waist_max_deg": 160.0})
        self.assertEqual(drv.limits_api()[0], (0.0, 400.0))
        self.assertEqual(drv.limits_api()[1], (-160.0, 160.0))


class TestCommandGate(unittest.TestCase):
    def test_read_only_rejects(self):
        drv, _ = _driver({"motion_enabled": False})
        drv._ingest_status(_axes())
        self.assertFalse(drv.command(["l_1"], [0.1]))
        self.assertEqual(drv._client.moveto_calls, [])

    def test_no_state_yet_rejects(self):
        drv, _ = _driver({"motion_enabled": True})
        self.assertFalse(drv.command(["l_1"], [0.1]))

    def test_unknown_joint_is_ignored(self):
        drv, _ = _driver({"motion_enabled": True})
        drv._ingest_status(_axes())
        self.assertFalse(drv.command(["arm_joint_1"], [0.1]))
        self.assertEqual(drv._client.moveto_calls, [])

    def test_out_of_limit_is_rejected(self):
        drv, _ = _driver({"motion_enabled": True})
        drv._ingest_status(_axes())
        self.assertFalse(drv.command(["l_1"], [0.5]))  # 500 mm > 300 mm
        self.assertEqual(drv._client.moveto_calls, [])

    def test_accepted_command_keeps_other_axes_in_place(self):
        drv, _ = _driver({"motion_enabled": True})
        drv._ingest_status(_axes(lift_mm=120.0, waist_deg=10.0))
        self.assertTrue(drv.command(["l_2"], [math.radians(30.0)]))
        positions, vel, acc = drv._client.moveto_calls[0]
        self.assertAlmostEqual(positions[0], 120.0)   # untouched lift
        self.assertAlmostEqual(positions[1], 30.0)    # commanded waist, in deg
        self.assertAlmostEqual(positions[2], 0.0)
        self.assertEqual((vel, acc), (10, 10))

    def test_partial_accept_when_one_target_is_out_of_range(self):
        drv, _ = _driver({"motion_enabled": True})
        drv._ingest_status(_axes())
        accepted = drv.command(["l_1", "l_4"], [0.1, math.radians(90.0)])
        self.assertTrue(accepted)
        positions, _, _ = drv._client.moveto_calls[0]
        self.assertAlmostEqual(positions[0], 100.0)
        self.assertAlmostEqual(positions[3], 0.0)  # 90 deg pitch rejected

    def test_short_zip_is_safe(self):
        drv, _ = _driver({"motion_enabled": True})
        drv._ingest_status(_axes())
        self.assertTrue(drv.command(["l_1"], [0.1, 0.2]))


class TestFeedback(unittest.TestCase):
    @unittest.skipUnless(HAS_ROS, "needs ROS 2 sensor_msgs")
    def test_joint_state_carries_the_urdf_names_and_ros_units(self):
        drv, published = _driver({"motion_enabled": False})
        drv._ingest_status(_axes(lift_mm=250.0, waist_deg=-90.0,
                                 head_yaw_deg=45.0, head_pitch_deg=10.0))
        message = published[-1]
        self.assertEqual(list(message.name), list(JOINT_NAMES))
        self.assertAlmostEqual(message.position[0], 0.25)
        self.assertAlmostEqual(message.position[1], -math.pi / 2)
        self.assertAlmostEqual(message.position[2], math.pi / 4)
        self.assertAlmostEqual(message.position[3], math.radians(10.0))
        self.assertEqual(message.header.frame_id, "base_link")

    def test_missing_axis_raises(self):
        drv, _ = _driver()
        with self.assertRaises(ExtAxisError):
            drv._ingest_status([{"id": 0, "pos": 0.0}])

    def test_successful_poll_clears_the_failure_counter(self):
        client = FakeExtClient()
        drv, published = _driver(client=client)
        drv._failures = 2
        self.assertTrue(drv._poll_once())
        self.assertEqual(drv._failures, 0)

    def test_failure_without_state_publishes_nothing(self):
        client = FakeExtClient()
        client.fail = True
        drv, published = _driver(client=client)
        self.assertFalse(drv._poll_once())
        self.assertEqual(published, [])

    @unittest.skipUnless(HAS_ROS, "needs ROS 2 sensor_msgs")
    def test_repeated_failure_republishes_the_last_known_values(self):
        client = FakeExtClient()
        drv, published = _driver(
            {"publish_last_known_on_error": True, "error_threshold": 2},
            client=client,
        )
        drv._ingest_status(_axes(lift_mm=100.0))
        self.assertEqual(len(published), 1)

        client.fail = True
        drv._poll_once()                      # 1 failure: below threshold
        self.assertEqual(len(published), 1)
        drv._poll_once()                      # 2 failures: stale republish
        self.assertEqual(len(published), 2)
        self.assertAlmostEqual(published[-1].position[0], 0.1)

    def test_stale_republish_can_be_disabled(self):
        client = FakeExtClient()
        drv, published = _driver(
            {"publish_last_known_on_error": False, "error_threshold": 1},
            client=client,
        )
        drv._ingest_status(_axes(lift_mm=100.0))
        published.clear()
        client.fail = True
        drv._poll_once()
        self.assertEqual(published, [])


class TestLifecycle(unittest.TestCase):
    def test_start_enables_and_is_idempotent(self):
        drv, _ = _driver({"auto_enable": True})
        drv._stop.set()  # keep the poll thread from looping in this test
        drv.start()
        drv.start()
        self.assertEqual(drv._client.enable_calls, [True])

    def test_auto_enable_can_be_turned_off(self):
        drv, _ = _driver({"auto_enable": False})
        drv._stop.set()
        drv.start()
        self.assertEqual(drv._client.enable_calls, [])

    def test_shutdown_stops_and_disables(self):
        drv, _ = _driver({"auto_enable": True, "motion_enabled": False})
        drv.shutdown()
        self.assertEqual(drv._client.stop_calls, 1)
        self.assertEqual(drv._client.enable_calls, [False])

    def test_shutdown_does_not_home_while_read_only(self):
        drv, _ = _driver(
            {"motion_enabled": False, "shutdown_home": [0.0, 0.0, 0.0, 0.0]}
        )
        drv.shutdown()
        self.assertEqual(drv._client.moveto_calls, [])


if __name__ == "__main__":
    unittest.main()
