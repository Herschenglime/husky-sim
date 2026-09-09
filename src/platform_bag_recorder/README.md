# platform_bag_recorder

Rosbag2 recorder for the Clearpath **a200** and the Unitree **go1**. Records
laser scans (2D and 3D), camera images (compressed when actually published),
`/tf`, odometry, `cmd_vel` and IMU — whatever the robot in front of it is
really publishing.

Built along the same lines as `turtlebot4_bag_recorder`: message types and
publisher QoS are discovered at runtime, so best-effort sensor topics are
actually received rather than silently dropped.

## Run

```bash
ros2 launch platform_bag_recorder record.launch.py platform:=go1
ros2 launch platform_bag_recorder record.launch.py platform:=a200
```

Ctrl+C stops it and finalizes the bag. Bags land in `~/bags/<platform>_<timestamp>`
as MCAP by default.

Useful arguments: `namespace` (override the profile default), `bag_name`,
`bag_base_dir`, `storage_id` (`mcap`/`sqlite3`), and `sync_rate_hz` — leave at
0 to record every message as it arrives, or set it to write all topics
synchronized at one uniform rate.

## What gets recorded

| Slot | a200 | go1 |
| --- | --- | --- |
| `scan_2d` | `sensors/lidar2d_0/scan` | `scan` |
| `scan_3d`, `points_3d` | `sensors/lidar3d_0/…` | — (no 3D lidar) |
| `camera` | `sensors/camera_0/color/image[/compressed]` | `color/image_raw[/compressed]` |
| `imu` | `sensors/imu_0/data` | `imu_plugin/out` |
| `odom` | `platform/odom/filtered` → `platform/odom` | `odometry/filtered` → `odom` |
| `cmd_vel` | `cmd_vel` | `cmd_vel` |
| `tf`, `tf_static` | `tf`, `tf_static` | `tf`, `tf_static` |

Each row is a *slot* with candidate topics in preference order. The first
candidate that qualifies wins, so the caller does not need to know the sensor
fit — which matters because the a200's navigation config deliberately drops the
sensor arch, and the camera and 3D lidar with it.

## Three things that are easy to get wrong

**Advertised is not published.** The go1 advertises
`color/image_raw/compressed` with a live publisher that emits nothing —
`image_transport` advertises a transport whether or not anything drives it.
Preferring compressed on the strength of the advertisement put an empty topic
in the bag while a perfectly good 10 Hz raw stream sat beside it. So sensor
slots are *probed*: the recorder subscribes to the candidates, sees which
actually deliver, and only then chooses. When it falls back it says why:

```
camera -> /robot1/color/image_raw  (preferred .../compressed is advertised but silent)
```

**Latched topics deliver once, on subscribe.** `/tf_static` is
`TRANSIENT_LOCAL` and arrives during probing, then never again — so choosing a
winner on that message and starting to record afterwards left `/tf_static` in
the bag with a count of zero. A bag with no static transforms cannot be
replayed into a working TF tree. The newest probed message for each winner is
flushed into the bag at the moment recording starts.

**`cmd_vel` is legitimately silent.** Nothing publishes it while the robot is
idle, so a rule of "only record what has delivered" would drop it, and making
it required would hang the recorder forever waiting. It is selected on
advertisement instead (`probe=False`), so it lands in the bag correctly typed
and empty, and fills in as soon as the robot is commanded.
