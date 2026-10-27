# SPDX-License-Identifier: Apache-2.0
"""JAKA Lumi body-joint driver.

Drives the four "ext-axis" motors (lift column, waist yaw, head yaw, head
pitch) over the vendor HTTP API and publishes them as ``sensor_msgs/JointState``
on ``/joint_states``.

Why this package exists at all: ``robot_state_publisher`` only derives TF from
the *latest* JointState message (verified against ROS 2 Humble), and both Lumi
cameras hang above the lift/waist joints. Without a publisher for ``l_1..l_4``
the camera frames never appear in the TF tree, and RGB-D mapping cannot start.

Units: the vendor speaks millimetres for the lift and degrees for the three
rotations; ROS/URDF expects metres and radians. Conversion happens exactly
here, at the provider boundary.
"""
from __future__ import annotations

import math
import threading
import time

from .http_client import ExtAxisError, LumiExtAxisClient

# URDF joint names, in vendor axis order.
JOINT_NAMES = ("l_1", "l_2", "l_3", "l_4")

# URDF limits per axis, in ROS units: the outer bound a deployment may opt into.
URDF_LIMITS = (
    (0.0, 0.4),                              # l_1 prismatic, m
    (math.radians(-160.0), math.radians(160.0)),   # l_2 revolute, rad
    (math.radians(-180.0), math.radians(180.0)),   # l_3
    (math.radians(-5.0), math.radians(35.0)),      # l_4
)

# Vendor API limits, the conservative default used unless the config relaxes it.
API_LIMIT_MM_LIFT = (0.0, 300.0)
API_LIMIT_DEG_WAIST = (-140.0, 140.0)
API_LIMIT_DEG_HEAD_YAW = (-180.0, 180.0)
API_LIMIT_DEG_HEAD_PITCH = (-5.0, 35.0)

_WARN_INTERVAL_S = 5.0


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# ── unit conversion ──────────────────────────────────────────────────────
def api_to_ros(values) -> tuple[float, float, float, float]:
    """[mm, deg, deg, deg] -> [m, rad, rad, rad]."""
    lift_mm, waist_deg, head_yaw_deg, head_pitch_deg = (float(v) for v in values)
    return (
        lift_mm / 1000.0,
        math.radians(waist_deg),
        math.radians(head_yaw_deg),
        math.radians(head_pitch_deg),
    )


def ros_to_api(values) -> tuple[float, float, float, float]:
    """[m, rad, rad, rad] -> [mm, deg, deg, deg]."""
    lift_m, waist_rad, head_yaw_rad, head_pitch_rad = (float(v) for v in values)
    return (
        lift_m * 1000.0,
        math.degrees(waist_rad),
        math.degrees(head_yaw_rad),
        math.degrees(head_pitch_rad),
    )


# ── config ───────────────────────────────────────────────────────────────
def _num(cfg: dict, key: str, default, lo=None, hi=None, integer=False):
    raw = cfg.get(key, default)
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"`{key}` must be a number, got {raw!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"`{key}` must be finite, got {raw!r}")
    if lo is not None and value <= lo:
        raise ValueError(f"`{key}` must be > {lo}, got {value}")
    if hi is not None and value >= hi:
        raise ValueError(f"`{key}` must be < {hi}, got {value}")
    return int(value) if integer else value


def _flag(cfg: dict, key: str, default: bool) -> bool:
    raw = cfg.get(key, default)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        lowered = raw.strip().lower()
        if lowered in ("1", "true", "yes", "on"):
            return True
        if lowered in ("0", "false", "no", "off", ""):
            return False
    raise ValueError(f"`{key}` must be a boolean, got {raw!r}")


def _text(cfg: dict, key: str, default: str) -> str:
    raw = cfg.get(key, default)
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError(f"`{key}` must be a non-empty string, got {raw!r}")
    return raw.strip()


def _pct(cfg: dict, key: str, default: int) -> int:
    """Vendor percentages are 1..100 inclusive."""
    value = _num(cfg, key, default, lo=0.0)
    if value > 100.0:
        raise ValueError(f"`{key}` must be <= 100, got {value}")
    return int(value)


def parse_config(cfg: dict) -> dict:
    """Validate the deployment entry's ``config`` mapping into a normalized dict."""
    cfg = dict(cfg or {})

    lift_max_mm = _num(cfg, "lift_max_mm", 300.0, lo=0.0)
    if lift_max_mm > 400.0:
        raise ValueError(
            f"`lift_max_mm` {lift_max_mm} exceeds the URDF lift travel 400 mm"
        )
    waist_max_deg = _num(cfg, "waist_max_deg", 140.0, lo=0.0)
    if waist_max_deg > 160.0:
        raise ValueError(
            f"`waist_max_deg` {waist_max_deg} exceeds the URDF waist limit 160 deg"
        )

    home = cfg.get("shutdown_home")
    if home is not None:
        if not isinstance(home, (list, tuple)) or len(home) != 4:
            raise ValueError("`shutdown_home` must be a list of 4 numbers")
        home = tuple(float(value) for value in home)

    return {
        "base_url": _text(cfg, "base_url", "http://192.168.10.90:5000").rstrip("/"),
        "http_timeout_s": _num(cfg, "http_timeout_s", 5.0, lo=0.0),
        "motion_enabled": _flag(cfg, "motion_enabled", False),
        "auto_enable": _flag(cfg, "auto_enable", True),
        "joint_states_topic": _text(cfg, "joint_states_topic", "/joint_states"),
        "joint_command_topic": _text(
            cfg, "joint_command_topic", "/lumi_body/joint_command"
        ),
        "state_publish_hz": _num(cfg, "state_publish_hz", 10.0, lo=0.0),
        "lift_max_mm": lift_max_mm,
        "waist_max_deg": waist_max_deg,
        "default_vel_pct": _pct(cfg, "default_vel_pct", 10),
        "default_acc_pct": _pct(cfg, "default_acc_pct", 10),
        "publish_last_known_on_error": _flag(
            cfg, "publish_last_known_on_error", True
        ),
        "error_threshold": _num(
            cfg, "error_threshold", 3, lo=0.0, integer=True
        ),
        "shutdown_home": home,
        "move_timeout_s": _num(cfg, "move_timeout_s", 20.0, lo=0.0),
    }


# ── driver ───────────────────────────────────────────────────────────────
class LumiBodyDriver:
    """One body. Owns the joint_states publisher and the command path."""

    def __init__(self, cfg: dict, publish_joint_states, clock=None, log=print):
        self.cfg = cfg
        self._publish = publish_joint_states
        self._clock = clock
        self._log = log
        self._client = LumiExtAxisClient(
            base_url=cfg["base_url"], timeout_s=cfg["http_timeout_s"]
        )

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self._last_api: tuple[float, float, float, float] | None = None
        self._last_enabled: tuple[bool, bool, bool, bool] | None = None
        self._failures = 0
        self._last_warn = 0.0
        self._last_reject = 0.0
        self.info: dict = {}

    # -- helpers ---------------------------------------------------------
    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warn >= _WARN_INTERVAL_S:
            self._last_warn = now
            self._log(f"WARNING {message}")

    def _stamp(self):
        if self._clock is None:
            return None
        return self._clock().now().to_msg()

    def limits_api(self) -> tuple[tuple[float, float], ...]:
        """Per-axis ``(low, high)`` in vendor units, after config relaxation."""
        return (
            (API_LIMIT_MM_LIFT[0], self.cfg["lift_max_mm"]),
            (-self.cfg["waist_max_deg"], self.cfg["waist_max_deg"]),
            API_LIMIT_DEG_HEAD_YAW,
            API_LIMIT_DEG_HEAD_PITCH,
        )

    # -- lifecycle -------------------------------------------------------
    def connect(self) -> dict:
        """Liveness + identity probe for CMD_INIT."""
        info = self._client.sysinfo()
        status = self._client.status()
        self.info = info
        self._ingest_status(status, publish=False)
        return info

    def start(self) -> None:
        if self._thread is not None:
            return  # idempotent ACTIVATE
        if self.cfg["auto_enable"]:
            self._enable(True)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._poll_loop, name="lumi-body-poll", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def shutdown(self) -> None:
        self.stop()
        home = self.cfg["shutdown_home"]
        if home is not None and self.cfg["motion_enabled"]:
            try:
                self._moveto(home, wait=True, timeout=self.cfg["move_timeout_s"])
            except Exception as exc:  # noqa: BLE001 - shutdown must not raise
                self._log(f"shutdown_home failed: {exc}")
        try:
            self._client.stop()
        except Exception as exc:  # noqa: BLE001
            self._log(f"stop failed: {exc}")
        if self.cfg["auto_enable"]:
            self._enable(False)

    def _enable(self, engaged: bool) -> None:
        try:
            self._client.enable(engaged)
        except ExtAxisError as exc:
            self._log(f"{'enable' if engaged else 'disable'} failed: {exc}")

    # -- feedback --------------------------------------------------------
    def _ingest_status(self, status: list[dict], publish: bool = True) -> None:
        by_id = {}
        for entry in status:
            if isinstance(entry, dict) and "id" in entry:
                by_id[int(entry["id"])] = entry
        positions = []
        for index in range(4):
            entry = by_id.get(index)
            if entry is None:
                raise ExtAxisError(f"status reply is missing axis {index}")
            positions.append(float(entry.get("pos", 0.0)))
        api_values = tuple(positions)

        enabled = tuple(bool(by_id[i].get("enable")) for i in range(4))
        faults = [
            (i, by_id[i].get("error"), by_id[i].get("ecode"))
            for i in range(4)
            if by_id[i].get("error") or by_id[i].get("ecode")
        ]

        with self._lock:
            self._last_api = api_values
            self._last_enabled = enabled

        if faults:
            self._warn(f"axes report faults: {faults}")

        if publish:
            self._publish_values(api_to_ros(api_values), stale=False)

    def _publish_values(self, positions, stale: bool) -> None:
        stamp = self._stamp()
        try:
            self._publish(self._make_joint_state(positions, stamp, stale))
        except Exception as exc:  # noqa: BLE001
            self._warn(f"joint_states publish failed: {exc}")

    def _make_joint_state(self, positions, stamp, stale: bool):
        from sensor_msgs.msg import JointState

        msg = JointState()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.name = list(JOINT_NAMES)
        msg.position = [float(value) for value in positions]
        # The vendor reports velocity in deg/s mixed with mm/s; publishing it
        # would mix unit systems inside one JointState, so leave it empty.
        return msg

    def _poll_loop(self) -> None:
        period = 1.0 / self.cfg["state_publish_hz"]
        while not self._stop.is_set():
            self._poll_once()
            self._stop.wait(period)

    def _poll_once(self) -> bool:
        """One status read + publish. Returns True when the read succeeded."""
        try:
            self._ingest_status(self._client.status())
            self._failures = 0
            return True
        except Exception as exc:  # noqa: BLE001 - keep the stream alive
            self._failures += 1
            self._warn(f"body status read failed ({self._failures}x): {exc}")
            with self._lock:
                last_known = self._last_api
            if (
                self.cfg["publish_last_known_on_error"]
                and last_known is not None
                and self._failures >= self.cfg["error_threshold"]
            ):
                # Keep the TF tree stable with the last measured values and let
                # the repeated warning carry the staleness.
                self._publish_values(api_to_ros(last_known), stale=True)
            return False

    # -- command ---------------------------------------------------------
    def command(self, names, positions) -> bool:
        """Handle one joint_command message. Returns True when accepted."""
        if not self.cfg["motion_enabled"]:
            now = time.monotonic()
            if now - self._last_reject >= _WARN_INTERVAL_S:
                self._last_reject = now
                self._log("joint_command rejected: motion_enabled is false")
            return False

        with self._lock:
            current = self._last_api
        if current is None:
            self._log("joint_command rejected: no body state yet")
            return False

        # Work in ROS units, then convert once: the vendor always needs all
        # four axes, so unlisted joints must keep their measured position.
        targets_ros = list(api_to_ros(current))
        limits = self.limits_api()
        touched: list[int] = []
        for name, raw in zip(names or (), positions or ()):
            try:
                index = JOINT_NAMES.index(str(name))
            except ValueError:
                self._log(f"joint_command ignored unknown joint {name!r}")
                continue
            targets_ros[index] = float(raw)
            touched.append(index)

        if not touched:
            return False

        targets_api = list(ros_to_api(targets_ros))
        accepted = 0
        for index in touched:
            low, high = limits[index]
            if low <= targets_api[index] <= high:
                accepted += 1
                continue
            self._log(
                f"joint_command rejected: {JOINT_NAMES[index]} target "
                f"{targets_api[index]:.2f} outside [{low:.2f}, {high:.2f}]"
            )
            targets_api[index] = current[index]

        if accepted == 0:
            return False

        try:
            self._moveto(targets_api, wait=False)
        except ExtAxisError as exc:
            self._log(f"moveto failed: {exc}")
            return False
        return True

    def _moveto(self, targets_api, wait: bool = False, timeout: float = 0.0) -> None:
        self._client.moveto(
            targets_api, self.cfg["default_vel_pct"], self.cfg["default_acc_pct"]
        )
        if not wait:
            return
        deadline = time.monotonic() + (timeout or self.cfg["move_timeout_s"])
        while time.monotonic() < deadline:
            if self._stop.is_set():
                return
            status = self._client.status()
            moving = any(abs(float(entry.get("vel", 0.0))) > 0.05 for entry in status)
            if not moving:
                self._ingest_status(status)
                return
            time.sleep(0.2)
