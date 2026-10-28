# primitive-jaka-lumi-body-rbnx

Robonix package for the JAKA Lumi **body joints**: it drives the lift column,
waist rotation, head rotation and head pitch — the four motors the vendor calls
"ext-axis" — and publishes them on `/joint_states`.

```text
lumi_body/
├── http_client.py   vendor HTTP API: sysinfo / status / enable / reset / stop / moveto
├── driver.py        config validation + unit conversion + status polling + limits + command dispatch
└── main.py          Primitive lifecycle (package-local contracts)
```

## Why this package has to exist

`robot_state_publisher` derives TF only from the **latest** `/joint_states`
message (verified on ROS 2 Humble). The Lumi kinematic chain is:

```text
base_link → l_1(lift) → l_2(waist) → { l_3(head yaw) → l_4(head pitch) → Camera3_Link
                                    { Camera2_Link
                                    { l_a1..l_a6(arm)
```

Both RGB-D cameras hang above the lift/waist/head joints. **Without a publisher
for `l_1..l_4` the cameras never enter the TF tree**, RTAB-Map cannot resolve
image-to-map transforms, and mapping does not start. The arm package only
publishes its own six joints on `/jaka/joint_states`, so it cannot fill this
chain; splitting the joints across two publishers makes them overwrite each
other and the TF tree flaps. This package therefore owns `/joint_states` and
publishes `l_1..l_4` on it.

## Capabilities

| Contract | Mode | Transport | Endpoint |
|---|---|---|---|
| `robonix/primitive/body/joint_states` | `topic_out` | ros2 | `/joint_states` |
| `robonix/primitive/body/joint_command` | `topic_in` | ros2 | `/lumi_body/joint_command` |

Neither contract exists in the global capability tree, so both are declared
locally in `capabilities/body/*.toml` (`packaging-spec.md §1.2` allows this;
`rbnx codegen` and Atlas merge package-level contracts automatically).

| Joint | URDF type | Default limit | Unit |
|---|---|---|---|
| `l_1` | prismatic | 0 … 300 mm (URDF allows 400) | m |
| `l_2` | revolute | ±140° (URDF allows 160) | rad |
| `l_3` | revolute | ±180° | rad |
| `l_4` | revolute | -5° … 35° | rad |

Details are in [CAPABILITY.md](CAPABILITY.md); the configuration fields are in
[config.spec](config.spec).

## Prerequisites

- Robonix installed with the source root registered: `rbnx setup ~/robonix`
- ROS 2 Humble (`/opt/ros/humble`) with `sensor_msgs` and `colcon`
- Reachable cabinet: `curl http://192.168.10.90:5000/api/extaxis/sysinfo`
  (failure is usually the missing `192.168.10.x/24` static address on the WiFi link)

## Build and test

```bash
rbnx validate .
bash scripts/build.sh          # venv + codegen + colcon build (about 15 minutes)

# unit tests (39, no hardware needed; the publish path needs the ROS environment)
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s lumi_body/tests -t .
```

## Standalone start (debugging)

```bash
rbnx start -p . --endpoint 127.0.0.1:50051 --set motion_enabled=false
```

`motion_enabled` defaults to `false`: only `/joint_states` is published and every
`joint_command` is rejected with a warning. Enable it once the feedback and the
limits look right:

```bash
rbnx start -p . --set motion_enabled=true --set lift_max_mm=300
```

## Verification

```bash
# 1) provider and capabilities
rbnx caps -v | rg -A6 lumi_body

# 2) are the four joints in the stream
ros2 topic echo /joint_states --once

# 3) the decisive check: did the cameras enter the TF tree
ros2 run tf2_ros tf2_echo base_link Camera3_Link
ros2 run tf2_ros tf2_echo base_link Camera2_Link
```

If the third check resolves, the chain "body package + whole-robot URDF +
robot_state_publisher" is connected.

## Known limitations

- **The six arm joints do not enter TF.** They are published by the reused
  upstream arm package on `/jaka/joint_states`. Completing the tree needs either
  `publish_arm_joints` (a second jkrc client reading the arm angles) or forking
  the arm package so one process publishes all ten joints. Both wait for the
  on-hardware acceptance run.
- **The `velocity` field stays empty.** The vendor reports mm/s and deg/s per
  axis; mixing those unit systems inside one JointState is worse than omitting
  them.
- **No tool/payload changes.** That belongs to the arm package, not to the body.
- After more than `error_threshold` consecutive status failures the provider keeps
  publishing the last measured values (with a warning) so the TF tree does not
  break. Set `publish_last_known_on_error: false` to go silent instead.

## License

Apache-2.0.
