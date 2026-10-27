# primitive-jaka-lumi-body-rbnx

JAKA Lumi **本体关节**原语的 Robonix 软件包：驱动升降柱、腰部旋转、头部旋转、
头部俯仰这 4 个电机（厂商称"外部轴"），并发布到 `/joint_states`。

```text
lumi_body/
├── http_client.py   厂商 HTTP 接口：sysinfo / status / enable / reset / stop / moveto
├── driver.py        配置校验 + 单位换算 + 状态轮询 + 限位 + 指令下发
└── main.py          Primitive 生命周期（包内本地契约）
```

## 这个包为什么必须存在

`robot_state_publisher` 只按**最新一条** `/joint_states` 生成 TF（已用 ROS 2
Humble 实测确认）。Lumi 的运动链是：

```text
base_link → l_1(升降) → l_2(腰) → { l_3(头旋) → l_4(头俯) → Camera3_Link
                                 { Camera2_Link
                                 { l_a1..l_a6(机械臂)
```

两台 RGB-D 相机都挂在升降/腰/头之上。**没有人发布 `l_1..l_4`，相机就不在 TF
树里**，RTAB-Map 拿不到"图像→地图"的变换，建图直接起不来。而机械臂包只发它
自己的 6 个关节到 `/jaka/joint_states`，补不上这条链；拆成两个 publisher 又会
互相覆盖导致 TF 抖动。所以由这个包独占 `/joint_states` 发布 `l_1..l_4`。

## 能力

| 契约 | 模式 | 传输 | 端点 |
|---|---|---|---|
| `robonix/primitive/body/joint_states` | `topic_out` | ros2 | `/joint_states` |
| `robonix/primitive/body/joint_command` | `topic_in` | ros2 | `/lumi_body/joint_command` |

两个契约在全局能力树里不存在，因此在本包 `capabilities/body/*.toml` 里本地定义
（`packaging-spec.md §1.2` 允许，`rbnx codegen` 与 Atlas 会自动合并）。

| 关节 | URDF 类型 | 默认限位 | 单位 |
|---|---|---|---|
| `l_1` | prismatic | 0 ~ 300 mm（URDF 允许 400） | m |
| `l_2` | revolute | ±140°（URDF 允许 160） | rad |
| `l_3` | revolute | ±180° | rad |
| `l_4` | revolute | -5° ~ 35° | rad |

细节见 [CAPABILITY.md](CAPABILITY.md)，配置字段见 [config.spec](config.spec)。

## 前置条件

- Robonix 已安装并登记源码根：`rbnx setup ~/robonix`
- ROS 2 Humble（`/opt/ros/humble`），含 `sensor_msgs`、`colcon`
- 控制柜可达：`curl http://192.168.10.90:5000/api/extaxis/sysinfo`
  （连不上多半是 WiFi 侧缺 `192.168.10.x/24` 静态地址）

## 构建与验证

```bash
rbnx validate .
bash scripts/build.sh          # venv + codegen + colcon build（约 15 分钟）

# 单元测试（39 个，无需硬件；发布路径需要 ROS 环境）
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s lumi_body/tests -t .
```

## 单独启动（调试）

```bash
rbnx start -p . --endpoint 127.0.0.1:50051 --set motion_enabled=false
```

`motion_enabled` 默认 `false`，此时只发布 `/joint_states`，`joint_command` 全部
拒绝并打 warning。确认反馈与限位无误后打开：

```bash
rbnx start -p . --set motion_enabled=true --set lift_max_mm=300
```

## 验证

```bash
# 1) provider 与能力
rbnx caps -v | rg -A6 lumi_body

# 2) 四个关节是否在流里
ros2 topic echo /joint_states --once

# 3) 关键检查：相机是否进 TF 树
ros2 run tf2_ros tf2_echo base_link Camera3_Link
ros2 run tf2_ros tf2_echo base_link Camera2_Link
```

第 3 条能解析，说明"本体包 + 整机 URDF + robot_state_publisher"这条链是通的。

## 已知限制

- **机械臂 6 个关节不进 TF**：它们由复用上游的臂包发布到 `/jaka/joint_states`。
  要让整棵树完整，需要 `publish_arm_joints`（第二个 jkrc 客户端读臂角度）或
  fork 臂包让单进程同时发 10 个关节。两者都留到上机验证后再定。
- **`velocity` 字段留空**：厂商按轴给 mm/s 与 deg/s，混在一个 JointState 里比
  不提供更糟。
- **不改 tool/payload**：这是臂包的职责，与本体无关。
- 状态读取失败超过 `error_threshold` 次后会继续发最后一次的测量值（打 warning），
  目的是不让 TF 断掉；可用 `publish_last_known_on_error: false` 改成静默。

## 许可

Apache-2.0。
