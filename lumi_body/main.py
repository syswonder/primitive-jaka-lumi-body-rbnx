# SPDX-License-Identifier: Apache-2.0
"""lumi_body — Robonix body-joint primitive for the JAKA Lumi.

Owns the two package-local ``robonix/primitive/body/*`` contracts:

  joint_states   topic_out  ROS 2 /joint_states        sensor_msgs/JointState
  joint_command  topic_in   /lumi_body/joint_command   sensor_msgs/JointState

The four "ext-axis" motors (lift column, waist yaw, head yaw/pitch) live on
the same cabinet as the arm but speak JSON over HTTP on port 5000. Publishing
them is what keeps the upper body — and therefore both camera frames — in the
TF tree; see CAPABILITY.md.
"""
from __future__ import annotations

import logging

from robonix_api import Err, Ok, Primitive

provider = Primitive(id="lumi_body", namespace="robonix/primitive/body")

# Imported AFTER Primitive(): its constructor puts the package's codegen output
# on sys.path.
from sensor_msgs.msg import JointState  # noqa: E402

from .driver import LumiBodyDriver, parse_config  # noqa: E402

log = logging.getLogger("lumi_body")
logging.basicConfig(level=logging.INFO, format="[lumi_body] %(levelname)s %(message)s")

_JOINT_STATES_CONTRACT = "robonix/primitive/body/joint_states"
_JOINT_COMMAND_CONTRACT = "robonix/primitive/body/joint_command"

_driver: LumiBodyDriver | None = None


def _emit_joint_states(msg) -> None:
    provider.emit(_JOINT_STATES_CONTRACT, msg)


def _on_joint_command(msg: JointState) -> None:
    if _driver is None:
        return
    _driver.command(msg.name, msg.position)


# ── Lifecycle ────────────────────────────────────────────────────────────
@provider.on_init
def init(cfg: dict):
    global _driver

    try:
        normalized = parse_config(cfg)
    except ValueError as exc:
        return Err(f"invalid config: {exc}")

    provider.create_publisher(
        contract_id=_JOINT_STATES_CONTRACT,
        topic=normalized["joint_states_topic"],
        msg_type=JointState,
        qos="reliable",
        description=(
            "Body joints: l_1 lift (m), l_2 waist, l_3 head yaw, l_4 head "
            "pitch (rad). Same message robot_state_publisher uses for TF."
        ),
    )
    provider.create_subscription(
        contract_id=_JOINT_COMMAND_CONTRACT,
        topic=normalized["joint_command_topic"],
        msg_type=JointState,
        callback=_on_joint_command,
        qos="reliable",
    )

    clock = None
    try:
        from robonix_api.ros import RosBackend

        clock = RosBackend.get().node.get_clock
    except Exception as exc:  # noqa: BLE001
        return Err(f"failed to start the ROS 2 backend: {exc}")

    driver = LumiBodyDriver(
        normalized,
        publish_joint_states=_emit_joint_states,
        clock=clock,
        log=lambda text: log.info("%s", text),
    )
    try:
        info = driver.connect()
    except Exception as exc:  # noqa: BLE001
        return Err(
            f"body controller unreachable at {normalized['base_url']} "
            f"(the ext-axis API is HTTP on the cabinet address, port 5000): {exc}"
        )

    _driver = driver
    log.info(
        "CMD_INIT ok: base_url=%s motion_enabled=%s serial=%s version=%s",
        normalized["base_url"],
        normalized["motion_enabled"],
        info.get("serial_number"),
        info.get("version"),
    )
    return Ok()


@provider.on_activate
def activate():
    if _driver is None:
        return Err("provider is not initialised")
    _driver.start()
    enabled = _driver._last_enabled
    log.info(
        "CMD_ACTIVATE ok: %.1f Hz, enabled=%s",
        _driver.cfg["state_publish_hz"],
        enabled,
    )
    if enabled is not None and not all(enabled):
        log.warning(
            "some body axes are not enabled: %s (check the cabinet power and "
            "the e-stop, then /api/extaxis/enable)",
            enabled,
        )
    return Ok()


@provider.on_deactivate
def deactivate():
    if _driver is not None:
        _driver.stop()
        log.info("CMD_DEACTIVATE ok")
    return Ok()


@provider.on_shutdown
def shutdown():
    if _driver is not None:
        _driver.shutdown()
        log.info("CMD_SHUTDOWN ok")
    return Ok()


if __name__ == "__main__":
    provider.run()
