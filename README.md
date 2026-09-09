# Line following and point-to-point navigation — Husky a200 and Unitree go1

Two robots, two navigation problems, one pair of coupled workspaces:

| | **Clearpath Husky a200** | **Unitree go1** |
|---|---|---|
| **Line following** | `line_follow_track` (this workspace) | `gazebo_sim` line-follow launch (`~/go_sim`) |
| **Point-to-point** | `nav_worlds` — SLAM/AMCL + Nav2 | `gazebo_sim` — SLAM + Nav2, same worlds |
| **Goal sender** | `nav_worlds/send_goal.py` | the same script, `--ns robot1` |
| **Run recording** | `platform_bag_recorder platform:=a200` | `platform_bag_recorder platform:=go1` |
| Namespace | `a200_0000` | `robot1` |
| `cmd_vel` | `geometry_msgs/TwistStamped` | `geometry_msgs/Twist` |
| Footprint | 0.99 x 0.67 m, skid-steer | ~0.65 x 0.30 m, 12-DOF trot gait |

Both line followers and both navigation bringups are the *same* code and the
same staging logic, ported across platforms — the a200 versions came first and
the go1 versions changed only the platform bits. Each port's docstring carries
the difference table, so a behaviour that looks robot-specific can be checked
against the other robot rather than guessed at.

Both navigation bringups and the a200 follower run headless by default; the go1
follower is the exception and shows the GUI unless told otherwise. The GUI costs
a large slice of a core, and under Nav2 on the go1 that slice is the difference
between the stack activating and stalling half way (§7).

---

## 1. The two workspaces

| Workspace | Holds |
|---|---|
| `~/ros2_ws` | a200 line following and navigation, the shared goal sender and bag recorder, the worlds both robots drive |
| `~/go_sim` | the go1 itself — description, 12-DOF gait controller, its line follower and its Nav2 bringup |

They are deliberately coupled: `go1_nav_bringup.sh` reads worlds from
`~/ros2_ws/install/nav_worlds/`, puts `line_follow_sim`'s track models on
`GZ_SIM_RESOURCE_PATH`, and tells you to send goals with `nav_worlds`'s
`send_goal.py`. **Both workspaces must be built** before the go1 stacks work,
even though only `~/go_sim` gets sourced to run them.

## 2. Requirements

Verified on this machine — the configuration everything below was run against:

| | Version |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| ROS | Jazzy |
| Gazebo | Harmonic (`gz sim` 8.11.0) |
| `ros-jazzy-clearpath-simulator` | 2.9.4 |
| `ros-jazzy-clearpath-desktop` | 2.9.1 |
| `ros-jazzy-nav2-bringup` | 1.3.12 |
| `ros-jazzy-slam-toolbox` | 2.8.5 |
| `ros-jazzy-ros-gz` | 1.0.22 |
| `ros-jazzy-pointcloud-to-laserscan` | 2.0.2 |

`clearpath-simulator` **must be 2.9.4 or newer**. 2.9.3 and earlier spawn
neither of the a200's controllers, because `control.launch.py`'s sim-mode loop
only takes controllers whose name contains `controller` and contains neither
`manager` nor `platform` — `joint_state_broadcaster` and
`platform_velocity_controller` each fail that filter, so `controller_manager`
comes up empty and the robot ignores `cmd_vel` with nothing in any log saying
why.

A working GL/EGL context is needed even headless, because cameras and lidars
still render. On NVIDIA, check `nvidia-smi` for a driver/kernel module mismatch
before suspecting these packages — that shows up as `gz sim` segfaulting on
startup. First run of any world downloads models from Gazebo Fuel, so it needs
internet and a few minutes; later runs are cached in `~/.gz/fuel`.

## 3. Setup

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-clearpath-simulator \
  ros-jazzy-clearpath-desktop ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-teleop-twist-keyboard python3-opencv python3-numpy python3-vcstool

# a200 side
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
vcs import src < dependencies.repos          # Clearpath sources pinned here
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# go1 side
mkdir -p ~/go_sim/src && cd ~/go_sim/src
git clone https://github.com/abutalipovvv/go_sim_py.git .
cd ~/go_sim
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Source `~/ros2_ws/install/setup.bash` for a200 work and
`~/go_sim/install/setup.bash` for go1 work; each chains `/opt/ros/jazzy`
underneath. `go1_nav_bringup.sh` sources what it needs itself.

### 3.1 The `~/clearpath/robot.yaml` slot — a200 only, read before running it

Clearpath's generator chain reads **one** robot description, from
`~/clearpath/robot.yaml`, and the two a200 stacks need different ones:

| Stack | Needs | Installed by |
|---|---|---|
| `line_follow_track` | a200 + RealSense pitched 0.45 rad at the floor | `install_robot_config.sh` (backs the old one up) |
| `nav_worlds` | a200 + sensor arch, 2D and 3D lidar | copy `nav_worlds/config/a200_sample.yaml` |

They cannot coexist, and this is the most common reason a stack that worked
yesterday comes up with no camera or no lidar today. The namespace comes from
the same file and is baked into every topic name, so a mismatch also shows up as
topics that simply are not there. To keep both, put each in its own directory
and pass `setup_path:=/path/to/dir/` — every a200 launch here takes it.

The go1 has no equivalent: its namespace comes from
`gazebo_sim/config/robots.yaml` (`robot1`), which both its launches read, so the
name is never written down twice.

---

## 4. Line following — a200

**What it does.** The Husky spawns on a yellow line painted on the floor of
Clearpath's warehouse and drives the 158.6 m loop from its forward-down
RealSense: threshold for yellow in HSV, take the centroid of the largest blob in
a band across the lower frame, steer proportionally to how far off centre it
sits. Ten 0.5 m boxes stand on the line; a Bug0 boundary-following behaviour off
the 2D lidar rounds each one and rejoins the track.

`line_follow_sim` is the world, the track decal and the track's ground truth;
`line_follower` is the node and a scoring harness.

```bash
# once, and again whenever you come back from a nav_worlds run
"$(ros2 pkg prefix line_follow_sim)/share/line_follow_sim/scripts/install_robot_config.sh"

ros2 launch line_follower follow.launch.py                # headless
ros2 launch line_follower follow.launch.py gui:=true      # Gazebo window
ros2 launch line_follower follow.launch.py rviz:=true     # RViz
ros2 launch line_follow_sim line_follow_sim.launch.py     # world only, no follower
```

The robot spawns on the line at (5.0, 13.26) and starts driving on its first
camera frame. Without the robot config the launch stops with an error rather
than starting a sim that cannot work.

### Score a run

Launches everything, drives for a set time, scores the driven path against the
painted centreline and tears the sim down. Exit status is the run's PASS/FAIL,
so it can gate a change.

```bash
"$(ros2 pkg prefix line_follower)/share/line_follower/scripts/run_scored_test.sh" 90 --min-coverage 0.2
```

A representative full lap:

```
duration            339.9 s over 3082 samples
path length         172.4 m  (track loop is 158.6 m)
loop coverage       100% (159 of 159 one-metre bins)
cross-track error   mean 0.060 m | median 0.042 m | p95 0.160 m | max 0.320 m
                    within 0.15 m: 93%   within 0.30 m: 100%
=== PASS ===
```

Ground truth is Gazebo's own pose stream, not odometry — the a200's skid-steer
odometry drifts on the order of a metre over a lap, which would be scored as
tracking error the follower never made.

### Tune

`line_follower/config/line_follower.yaml`, read at startup — `ros2 param set`
has no effect, edit and relaunch.

| Parameter | Does |
|---|---|
| `forward_speed` | Cruise speed (0.6 m/s) |
| `k_steer` | Steering gain on centroid error |
| `roi_top` / `roi_bottom` | Which band of the frame is searched |
| `hue_min` / `hue_max` / `sat_min` | Yellow threshold, OpenCV hue 0–179 |
| `corridor_half` / `obstacle_trigger` | Width and range of the corridor swept for obstacles |
| `wall_distance` / `wall_speed` | Standoff and speed during a detour |
| `min_distance_from_hit` / `wall_follow_timeout` / `rearm_sec` | When a detour may end, and when a new one may start |

The obstacle-avoidance parameters are one interlocking set — the YAML records
what each wrong combination did, and changing one alone deadlocks the detour.

### Change the track

Centreline CSV, decal texture and decal mesh are generated together from one
drawing (`track_source/track_albedo.jpg`, black on white, 1006x1674 px matching
the 30x50 m floor 1:1). Editing one by hand silently misplaces the track.

```bash
cd "$(ros2 pkg prefix line_follow_sim)/share/line_follow_sim"
python3 scripts/gen_track_texture.py \
  --src track_source/track_albedo.jpg \
  --dst models/line_track/materials/textures/track_yellow.png \
  --mesh models/line_track/meshes/track_plane.obj \
  --centerline config/track_centerline.csv --width 0.15
python3 scripts/place_obstacles.py       # re-place the boxes against measured clearance
```

With `--symlink-install` that share path is the source tree; without it, run
against `src/` and rebuild or the regenerated files are thrown away. The track as
shipped clears every warehouse prop by at least 0.87 m — `shelf_big_2` is
deliberately absent from the world because the line passed 0.60 m from it,
leaving only 0.25 m of body margin.

### Known limits

- **Corner overshoot.** ~0.04 m on straights, ~0.32 m in tight turns: the
  centroid is taken over the whole ROI band, so on a curve the blob is
  banana-shaped and its centroid sits inside the turn. A narrow scanline row
  instead of the full band is the highest-value fix — more effective than
  raising `k_steer`.
- **`a200_0000` is hardcoded** in the follower config and the test script.
- **Scoring cost grows with run length** — a samples x 4979 distance matrix,
  ~120 MB for a 340 s run.

---

## 5. Line following — go1

**What it does.** The same algorithm on the quadruped: same HSV threshold, same
centroid steering, same Bug0 detour around obstacles. `line_follower.py` in
`gazebo_sim` is a port of the Husky follower and says so in its docstring, with
the platform differences in a table beside it.

```bash
source ~/go_sim/install/setup.bash
ros2 launch gazebo_sim line_follow.launch.py                      # its own line world, Gazebo GUI on
ros2 launch gazebo_sim line_follow.launch.py headless:=true      # no GUI
ros2 launch gazebo_sim line_follow.launch.py enable_rviz:=true
```

This is the one stack that shows the GUI by default — `line_follow.launch.py`
does not pass `headless` down, so `launch.py`'s own default (GUI on) stands;
`headless:=true` on the command line reaches it and turns the GUI off.

Nav2 and RViz are off by default here **by design**: Nav2 publishes to the same
`cmd_vel` the follower drives, and two controllers on one topic is not a useful
test. The robot spawns on a 6 s timer and the follower starts at 20 s, so its
first command cannot land before the gait controller is up.

To drive the Husky's warehouse track instead of the go1's own line world, point
it at that SDF — the track models have to be reachable, so set the resource path
too:

```bash
LFS=~/ros2_ws/install/line_follow_sim/share/line_follow_sim
export GZ_SIM_RESOURCE_PATH="$LFS/worlds:$LFS/models:$GZ_SIM_RESOURCE_PATH"
ros2 launch gazebo_sim line_follow.launch.py world_file:=$LFS/worlds/warehouse_line.sdf
```

### What changed in the port

| | a200 | go1 |
|---|---|---|
| namespace | `a200_0000` | `robot1` |
| `cmd_vel` | `TwistStamped` | `Twist` |
| `forward_speed` | 0.6 m/s | 0.5 — a 0–1 gait throttle, not m/s |
| `k_steer` | 1.5 | 0.9 — the dog yaws faster than a skid-steer |
| `max_angular` | 1.2 | 0.8 — `cmd_vel_pub` clamps angular.z to ±1 rad/s |
| camera | pitched 0.45 rad at the floor | fixed to the trunk, horizontal |
| `roi_top` / `roi_bottom` | 0.55 / 0.85 | 0.72 / 0.92 — the band is already near |
| `min_area_px` | 300 | 150 — smaller camera footprint on the paint |
| `lost_coast_sec` | 0.5 | 1.0 — a trot stride can drop the line for a frame or two |
| `wall_distance` | 0.70 | 0.50 — 0.25 box half + 0.15 body half + margin |
| `front_stop_distance` | 0.85 | 0.55 — body is ~0.65 m long, not 0.99 m |

Parameters are node parameters, so pass them with `--ros-args -p` or edit the
launch's `parameters=[...]`.

Two findings from the a200 work that the port keeps, and that are worth not
re-learning: the leave test may only contain terms the robot can *always*
eventually satisfy (gating it on a pre-detour heading, a side beam that reads
the warehouse, or a distance a pinned robot cannot travel each produced a robot
that circled until it timed out); and a camera sees a stripe, not an arrow —
nothing in the detection says which way along the track the robot points, so the
odometry heading rejects a line reacquired facing backwards. The reversal check
was measured on the go1: 3 reversals in 200 s when the heading reference was
allowed to expire, and reversals, not slow progress, are what hold coverage down.

---

## 6. Point-to-point — a200 (`nav_worlds`)

**What it does.** SLAM or AMCL plus Nav2 across the six worlds shipped with
`clearpath_gz` (`warehouse`, `office`, `construction`, `solar_farm`, `orchard`,
`pipeline`) plus `depot`, carried here. The navigation is Clearpath's
`clearpath_nav2_demos`; this package supplies what was missing to make it work —
staged bringup, a self-scan filter, a world launch without the `choices`
restriction, and a fixed depot world.

```bash
cp ~/ros2_ws/src/nav_worlds/config/a200_sample.yaml ~/clearpath/robot.yaml

ros2 run nav_worlds bringup.sh warehouse              # SLAM, builds a map as it drives
ros2 run nav_worlds bringup.sh warehouse --localize   # AMCL against maps/warehouse.yaml
HEADLESS=false ros2 run nav_worlds bringup.sh office  # with the Gazebo GUI
```

`bringup.sh` prints each stage as it becomes ready and exits non-zero if one
never does. Every wait is on observable state, not a fixed delay: the world, the
robot entity, a settled RTF, the lidar, active controllers, a `/map`, and finally
the `navigate_to_pose` action.

### Send goals

```bash
ros2 run nav_worlds send_goal.py 4.0 0.0        # x, y in the map frame
ros2 run nav_worlds send_goal.py 4.0 2.0 90     # x, y, yaw in degrees
ros2 run nav_worlds send_goal.py 4.0 2.0 --timeout 240 --ns a200_0000
```

It reports SUCCEEDED/FAILED with the time taken and how close the robot actually
got, so a run can be scored rather than eyeballed.

### Save a map for `--localize`

`maps/` ships empty, and `--localize` fails with a clear message until a map is
there. Drive the world under SLAM, then:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/nav_worlds/maps/warehouse \
  --ros-args -p use_sim_time:=true -r /map:=/a200_0000/map
colcon build --packages-select nav_worlds --symlink-install
```

The rebuild matters — `bringup.sh` reads the map from the *install* share
directory, not from `src`.

### Why this package exists

- **The lidar sees the robot, and Nav2 believes it.** With the sensor arch
  fitted, its legs sit ~0.40 m from the 2D lidar, inside the 1.1 x 0.9 m
  footprint. 32 of 720 beams return the robot's own structure, and Nav2's
  `collision_monitor` (`FootprintApproach`, `min_points: 12`) reads them as a
  permanently imminent collision and scales every command to zero — the robot
  accepts the goal, plans a path, logs "Passing new path to controller", and
  never moves. `scan_self_filter.py` drops returns whose *endpoint* lands inside
  the footprint; the closest return goes from 0.401 m to 1.099 m and the 683
  beams that see the world pass through byte-identical. Testing the endpoint is
  what makes it work — the lidar sits at x=0.328 m, already inside the footprint,
  so masking whole *bearings* would mask all 720 beams.
- **Nothing may start until the sim is at speed** (§9).
- **`clearpath_gz`'s `simulation.launch.py` only accepts its own six worlds** —
  its `world` argument carries a `choices` list, so `depot`, or any path, is
  rejected before Gazebo is reached. `launch/sim.launch.py` composes the same
  `gz_sim` + `robot_spawn` without that restriction.
- **`clearpath_gz` cannot run headless** — it builds `gz_args` itself and exposes
  no hook. `sim.launch.py` takes `headless:=true` and calls `ros_gz_sim`
  directly.
- **Every shipped copy of the depot world has the Sensors system commented out**,
  so the 2D lidar publishes nothing at all. `worlds/depot.sdf` enables `Sensors`
  and adds `Imu`, which the EKF needs for `odom -> base_link`.

---

## 7. Point-to-point — go1 (`gazebo_sim`)

**What it does.** The same job for the quadruped, staged the same way and in the
same worlds, so an a200 run and a go1 run are comparable.

```bash
~/go_sim/src/gazebo_sim/scripts/go1_nav_bringup.sh warehouse
# then, from a shell with ~/ros2_ws sourced:
ros2 run nav_worlds send_goal.py 4.0 0.0 --ns robot1
```

Worlds: `clearpath_gz`'s six resolve by name, `depot` comes from
`~/ros2_ws/install/nav_worlds/`, and `gazebo_sim`'s own (`cafe`, `office_small`,
`office_env_large`, `warehouse`, `line_follow`) work by bare name. The script
resolves the file, sets `GZ_SIM_RESOURCE_PATH` across both workspaces, and waits
for the world, the robot (`robot1_my_bot` in Gazebo), a settled RTF, the lidar,
a `/map` and the `navigate_to_pose` action before declaring READY.

Underneath it runs:

```bash
ros2 launch gazebo_sim launch.py \
  world:=warehouse world_file:=<path> enable_nav2:=true slam:=True \
  enable_rviz:=false headless:=true
```

`slam` takes `True`/`False` capitalized — the other flags take lowercase
`true`/`false`. `enable_nav2`, `slam` and `enable_rviz` are declared one level
down in `gazebo_multi_nav2_world.launch.py` and inherited from the command line,
which is why they do not appear in `launch.py --show-args`.

**Headless is not cosmetic here.** `launch.py` originally had no headless option
at all, so the GUI always ran at ~76% of a core. On top of the 12-DOF gait
controller, `gz_ros2_control`, SLAM and Nav2, that is enough to starve Nav2's
lifecycle activation: the manager gets partway through — "Activating
planner_server" — and stalls, so goals are rejected by a stack that looks like it
came up fine.

---

## 8. Recording runs — either robot

`platform_bag_recorder` records whichever of the two is in front of it, choosing
topics per *slot* from candidates in preference order, with types and publisher
QoS discovered at runtime so best-effort sensor topics are actually received.

```bash
ros2 launch platform_bag_recorder record.launch.py platform:=a200
ros2 launch platform_bag_recorder record.launch.py platform:=go1
```

Bags land in `~/bags/<platform>_<timestamp>` as MCAP. Useful arguments:
`namespace` (defaults `a200_0000` / `robot1`), `bag_name`, `bag_base_dir`,
`storage_id`, and `sync_rate_hz` (0 records every message as it arrives).

| Slot | a200 | go1 |
|---|---|---|
| `scan_2d` | `sensors/lidar2d_0/scan` | `scan` |
| `scan_3d`, `points_3d` | `sensors/lidar3d_0/…` | — no 3D lidar |
| `camera` | `sensors/camera_0/color/[compressed\|image]` | `color/image_raw[/compressed]` |
| `imu` | `sensors/imu_0/data` | `imu_plugin/out` |
| `odom` | `platform/odom/filtered` → `platform/odom` | `odometry/filtered` → `odom` |
| `cmd_vel`, `tf`, `tf_static` | same names | same names |

Three things it handles that are easy to get wrong: **advertised is not
published** (the go1 advertises a compressed camera transport that emits nothing,
so sensor slots are probed and the fallback says why), **latched topics deliver
once on subscribe** (`/tf_static` arrives during probing and never again, so the
newest probed message is flushed in when recording starts — a bag with no static
transforms cannot be replayed into a working TF tree), and **`cmd_vel` is
legitimately silent** while a robot is idle, so it is selected on advertisement
and lands correctly typed and empty.

---

## 9. Failure modes common to both robots

**Nothing works and the world looks fine — check the real-time factor.** A heavy
world sits near RTF 0.0005 while Gazebo loads meshes, and anything started in
that window dies in a way that does not name the cause: the controller spawner
hits `Switch controller timed out after 5 seconds` and dies, slam_toolbox logs
`Configuring` and never activates, Nav2 reports `failed to send response to
change_state` and strands its servers. Both bringup scripts gate on this.

```bash
gz topic -e -t /world/warehouse/stats -n 1 | grep real_time_factor
```

**A world's internal name is not always its filename.** `construction.sdf`
declares `<world name="office_construction">`, and Gazebo's topics use the
internal name — watching `/world/construction/...` never resolves, and a bringup
sits at its first gate while the simulation is running perfectly.

**The robot ignores `cmd_vel`.** On the a200, check what the controller manager
is actually running; `platform_velocity_controller` must be `active`:

```bash
ros2 service call /a200_0000/controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers
```

If it is missing, either `clearpath-simulator` is older than 2.9.4 (§2) or the
spawner was starved at startup — `bringup.sh` re-spawns it, `line_follow_sim`
tops it up on a timer via `ensure_controllers.py`.

**Sensors publish nothing but their topics exist.** Headless Gazebo needs
`--headless-rendering` in addition to `-s`; every launch here passes both, a
hand-rolled `gz sim -s` does not. The failure is silent — the topic is
advertised and never delivers.

**Topics missing or under an unexpected prefix.** The a200's namespace comes from
`~/clearpath/robot.yaml` (§3.1) — you are probably running one stack against
another's config. The go1's comes from `gazebo_sim/config/robots.yaml`.

**`import cv2` fails with `numpy.core.multiarray failed to import`.** A pip numpy
in `~/.local` is shadowing the apt one `python3-opencv` was built against. Remove
it (`pip uninstall --break-system-packages numpy`) or install a numpy-2
compatible wheel (`pip install --user --no-deps --break-system-packages
opencv-python-headless`).

**`gz sim` segfaults on startup.** Check `nvidia-smi` for a driver/kernel module
mismatch before suspecting these packages.

---

## 10. Layout

| Path | What it is |
|---|---|
| `~/ros2_ws/src/line_follow_track/line_follow_sim/` | Warehouse + painted track, track ground truth, obstacle placement |
| `~/ros2_ws/src/line_follow_track/line_follower/` | a200 OpenCV follower, scoring harness |
| `~/ros2_ws/src/nav_worlds/` | a200 Nav2/SLAM bringup, self-scan filter, `send_goal.py`, depot world |
| `~/ros2_ws/src/platform_bag_recorder/` | Bag recorder with a200 and go1 profiles |
| `~/ros2_ws/src/clearpath_common/`, `clearpath_config/`, `clearpath_msgs/` | Clearpath sources from `dependencies.repos` |
| `~/go_sim/src/gazebo_sim/` | go1 worlds, gait/Nav2 launches, `line_follower.py`, `go1_nav_bringup.sh` |
| `~/go_sim/src/go1_description/`, `go2_description/` | Quadruped descriptions |
| `~/go_sim/src/quadropted_controller/`, `quadropted_msgs/` | 12-DOF gait controller and its interface |

Package READMEs carry the detail this file summarizes:
`line_follow_track/README.md`, `nav_worlds/README.md`,
`platform_bag_recorder/README.md`.

## 11. Also in this workspace

Not part of the a200/go1 line-following and point-to-point work, but living
here and documented in their own READMEs:

| Package | What it is |
|---|---|
| `bug0_a300` | Bug0 reactive point-to-point for the Clearpath A300 Observer, off a flattened 3D lidar. Needs `a300_observer_sim.launch.py rviz_map:=true` for `map -> odom` |
| `bug0_turtlebot4` | Bug0 and Bug2 for the TurtleBot4, off `/scan` — the original the A300 and go1 controllers were ported from |
| `turtlebot4_gz_bringup_overlay` | Colorized maze world, TB4 spawn/SLAM/Nav2 launches |
| `husky_bamboo_sim`, `husky_culm_detection`, `husky_exploration` | Bamboo-field mission stack for the A300 |
