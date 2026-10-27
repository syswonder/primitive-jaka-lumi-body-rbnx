---
description: Drive the JAKA Lumi upper body — lift column, waist yaw, head yaw and head pitch — and publish them on /joint_states so the camera frames stay in the TF tree. Motion gated off by default.
---

# lumi_body capability notes

Provider ID is the deployment instance `name` (default `lumi_body`). The two
contracts are **package-local** (`capabilities/body/*.toml`) because the
Robonix capability tree has no body / ext-axis family; Atlas merges them at
register time, so consumers still just query
`robonix/primitive/body/joint_states`.

## Joints, frames and units

| Joint | URDF type | Range (config default) | Published unit |
|---|---|---|---|
| `l_1` | prismatic | 0 … `lift_max_mm` (default 300, URDF allows 400) | **m** |
| `l_2` | revolute | ±`waist_max_deg` (default 140, URDF allows 160) | **rad** |
| `l_3` | revolute | ±180° | **rad** |
| `l_4` | revolute | -5° … +35° | **rad** |

The vendor API speaks **millimetres** for the lift and **degrees** for the
rotations. Conversion happens exactly at this boundary: `api_to_ros` on the way
out, `ros_to_api` on the way in. Never send degrees to a consumer.

`joint_states.header.frame_id` is `base_link` (the URDF root). The joint names
must match `urdf/jaka_lumi.urdf` verbatim — `robot_state_publisher` matches by
name, and a rename silently drops that segment from the TF tree.

## Call order

1. `CMD_INIT` validates the config, declares both endpoints and calls
   `/api/extaxis/sysinfo` + `/api/extaxis/status`. An unreachable controller
   fails init (hard dependency, no `Deferred`).
2. `CMD_ACTIVATE` sends `enable {enable: 1}` when `auto_enable`, then starts
   polling at `state_publish_hz` (default 10 Hz). Repeating it is idempotent.
3. Publish `sensor_msgs/JointState` to `/lumi_body/joint_command` to move.
4. `CMD_DEACTIVATE` stops polling. `CMD_SHUTDOWN` optionally honours
   `shutdown_home`, sends `/api/extaxis/stop`, then disables.

## Command semantics

- Only the joints named in `JointState.name` are commanded; the rest keep their
  **current measured** position, because the vendor `moveto` always takes all
  four axes.
- Units follow the feedback stream: metres for `l_1`, radians for `l_2..l_4`.
- Unknown joint names are logged and skipped.
- Out-of-range targets are **rejected with a log line, not clamped** — a
  silently clamped lift limit hides a real programming error.
- A command is accepted when at least one axis passes; the reply is a single
  `/api/extaxis/moveto` with `default_vel_pct` / `default_acc_pct`.
- Commands are refused entirely while `motion_enabled` is false.

## Stale-feedback policy

A status read failure increments a counter. Past `error_threshold` (default 3)
the provider **keeps publishing the last measured values** and warns every 5 s,
because silence removes the upper body — and therefore both camera frames —
from the TF tree, which makes Mapping drop out. Set
`publish_last_known_on_error: false` if you would rather lose the transform
than see a stale one.

The published `velocity` field is intentionally empty: the vendor reports
mm/s and deg/s per axis, and mixing those unit systems inside one JointState
would be worse than omitting them.

## Typical errors

| Symptom | Cause |
|---|---|
| init fails with `body controller unreachable` | cabinet powered off, e-stop pressed, or the host has no `192.168.10.x/24` address |
| `some body axes are not enabled` at activate | servos not ready; power the cabinet and release the e-stop, then `POST /api/extaxis/enable` |
| `joint_command rejected: motion_enabled is false` | safety gate still on |
| `joint_command rejected: l_1 target 500.00 outside [0.00, 300.00]` | target above `lift_max_mm` (or above the URDF travel) |
| TF shows the waist but not the head | `l_3`/`l_4` missing from the message, or the URDF joint names were renamed |
