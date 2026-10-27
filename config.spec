# Runtime config accepted by the JAKA Lumi body-joint primitive.
#
# This file documents the `config:` mapping of this package's deployment
# entry in robonix_manifest.yaml. It is documentation for deployers and
# tooling; the provider re-validates every value in on_init and returns a
# located error instead of silently clamping.

config:
  # string, default: http://192.168.10.90:5000.
  # Vendor ext-axis HTTP base URL. Same cabinet as the arm, different port and
  # protocol — the arm speaks the jkrc SDK on TCP, these four motors speak JSON
  # over HTTP.
  base_url: http://192.168.10.90:5000

  # float, seconds, default: 5.0; must be > 0.
  # Per-request HTTP timeout. The status call is polled at state_publish_hz, so
  # a timeout longer than one period just delays the next poll.
  http_timeout_s: 5.0

  # bool, default: false. SAFETY GATE.
  # When false the provider only publishes joint_states and rejects every
  # joint_command with a rate-limited warning. Boot read-only, verify the
  # feedback and limits, then enable.
  motion_enabled: false

  # bool, default: true.
  # On CMD_ACTIVATE, send /api/extaxis/enable {enable: 1}. The vendor reports
  # success even when servos are not ready, so the first status read decides:
  # axes that stay disabled are logged as a warning.
  auto_enable: true

  # string, default: /joint_states.
  # Must be the topic robot_state_publisher subscribes to. Do not rename unless
  # the deployment also remaps the description primitive.
  joint_states_topic: /joint_states

  # string, default: /lumi_body/joint_command.
  # Command inlet for the body joints.
  joint_command_topic: /lumi_body/joint_command

  # float, Hz, default: 10.0; must be >= 1.0.
  # Status poll and joint_states publish rate. Each poll is one HTTP request.
  state_publish_hz: 10.0

  # float, mm, default: 300.0; must be > 0 and <= 400.
  # Lift travel limit. The manual advertises 0..400 mm but the HTTP API
  # documents 0..300 mm; the conservative value is the default and the URDF
  # allows up to 400 (urdf/jaka_lumi.urdf, joint l_1).
  lift_max_mm: 300.0

  # float, deg, default: 140.0; must be > 0 and <= 160.
  # Waist yaw limit. Same story: manual ±160°, API ±140°, URDF 2.7925 rad.
  waist_max_deg: 140.0

  # int, 1..100, default: 10.
  # Default speed percentage forwarded to /api/extaxis/moveto (vel field).
  # 10 is deliberately slow for a first bring-up; the vendor caps at 100.
  default_vel_pct: 10

  # int, 1..100, default: 10.
  # Default acceleration percentage (acc field).
  default_acc_pct: 10

  # bool, default: false.
  # Publish the last known joint values when a status read fails, instead of
  # going silent. Silence removes the upper body (and therefore both camera
  # frames) from the TF tree, which makes Mapping drop out; a stale-but-labelled
  # value keeps the tree stable while the warning tells the operator.
  publish_last_known_on_error: true

  # int, default: 3; must be >= 1.
  # Consecutive status failures before the stale state is announced.
  error_threshold: 3

  # list of 4 floats, optional, default: unset (no motion on exit).
  # On CMD_SHUTDOWN, move the body to [lift_mm, waist_deg, head_yaw_deg,
  # head_pitch_deg] before stopping. Only honoured while motion_enabled is
  # true. Bounded by move_timeout_s; failures are logged and shutdown proceeds.
  # shutdown_home: [0.0, 0.0, 0.0, 0.0]

  # float, seconds, default: 20.0; must be > 0.
  # Timeout for the optional shutdown_home move.
  move_timeout_s: 20.0
