# Warehouse line-following sim

A Clearpath Husky (a200) in the Gazebo warehouse, following a bright yellow line
painted on the floor, using OpenCV on the robot's camera. Two packages:

- **`line_follow_sim`** — the world: Clearpath's warehouse plus a painted track
  decal, the robot spawn, and the track's ground-truth geometry.
- **`line_follower`** — the follower node, and a harness that scores a run
  against the painted line.

## Requirements

- ROS 2 Jazzy on Ubuntu 24.04
- `ros-jazzy-clearpath-simulator` and `ros-jazzy-ros-gz` (Gazebo Harmonic)
- `python3-opencv`, `python3-numpy`
- A Clearpath setup directory, by default `~/clearpath`
- Internet on first run: the warehouse and shelf models download from Gazebo Fuel

## Install

Drop this folder into your workspace's `src/`, then:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Then install the robot config.** The follower needs a camera aimed at the
floor, and on a Clearpath robot that lives in `~/clearpath/robot.yaml`, outside
these packages — so a fresh checkout has lidar only and the follower would never
see anything. This copies in the config, backing up whatever is already there:

```bash
"$(ros2 pkg prefix line_follow_sim)/share/line_follow_sim/scripts/install_robot_config.sh"
```

It mounts an Intel RealSense on `sensor_arch_mount`, 0.30 m forward and pitched
0.45 rad down, which puts floor in the frame from ~0.8 m to ~7 m ahead — no
horizon, no chassis. If you skip this step the launch stops with an error saying
so rather than starting a sim that cannot work.

## Run

```bash
ros2 launch line_follower follow.launch.py
```

Headless by default; add `gui:=true` for the Gazebo window or `rviz:=true` for
RViz. The robot spawns on the line at (5.0, 13.26) and starts driving once it
has its first camera frame.

World only, no follower:

```bash
ros2 launch line_follow_sim line_follow_sim.launch.py
```

## Scoring a run

The harness launches everything, drives for a set time, scores the path against
the painted centreline, and tears the sim down. Exit status is PASS/FAIL.

```bash
"$(ros2 pkg prefix line_follower)/share/line_follower/scripts/run_scored_test.sh" 90 --min-coverage 0.2
```

A representative result (340 s, one full lap):

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

## Changing the track

`config/track_centerline.csv`, the decal texture and the decal mesh are all
generated from one drawing, `track_source/track_albedo.jpg` — a black stroke on
white, drawn at 1006 x 1674 px to match the 30 x 50 m warehouse floor 1:1
(0.0298 m/px, image top = +y). To change the track, redraw it at the same size
and regenerate all three together:

```bash
cd "$(ros2 pkg prefix line_follow_sim)/share/line_follow_sim"
python3 scripts/gen_track_texture.py \
  --src track_source/track_albedo.jpg \
  --dst models/line_track/materials/textures/track_yellow.png \
  --mesh models/line_track/meshes/track_plane.obj \
  --centerline config/track_centerline.csv \
  --width 0.15
```

Run it against your source tree, not the install directory, then rebuild. Do not
hand-edit the .obj or the .png: the floor extent lives in the script's
`FLOOR_X_M`/`FLOOR_Y_M`, and editing one file alone silently misplaces the track.

Check clearance after redrawing. The track as shipped clears every prop by at
least 0.87 m; `shelf_big_2` is deliberately absent from the world because the
line passed 0.60 m from it, leaving only 0.25 m of body margin.

## Tuning the follower

`line_follower/config/line_follower.yaml`. The useful knobs:

| Parameter | Does |
|---|---|
| `forward_speed` | Cruise speed (0.6 m/s) |
| `k_steer` | Steering gain on centroid error |
| `curve_slowdown` | How much to slow in a turn |
| `roi_top` | Fraction of frame ignored at the top — raising it looks nearer |
| `hue_min` / `hue_max` / `sat_min` | Yellow threshold, OpenCV hue 0-179 |
| `detection_timeout` | Age past which a detection stops counting as valid |
| `lost_coast_sec` / `search_angular` | Coast, then rotate-search, on line loss |

Parameters are read at startup, so `ros2 param set` will not take effect — edit
the YAML and relaunch.

## Known limitations

- **Corner overshoot.** Error is ~0.04 m on straights and peaks near 0.32 m in
  tight turns. The centroid is taken over the whole ROI band, so on a curve the
  blob is banana-shaped and its centroid sits inside the turn. Using a narrow
  scanline row instead of the full band is the highest-value change if you need
  tighter corners — more effective than raising `k_steer`.
- **No obstacle avoidance.** Clearance is a property of the track layout, checked
  when the track is drawn. The robot will drive into anything standing on the line.
- **The namespace `a200_0000` is hardcoded** in the follower config and the test
  script. Change both if your `robot.yaml` uses a different one.
- **`~/clearpath/robot.yaml` is a single global slot**, and this sim needs the
  a200 config in it. If you also run a sim for another Clearpath platform (an
  a300, say) the two configs cannot coexist there — re-run
  `install_robot_config.sh` when you come back to this one, and restore from the
  timestamped `robot.yaml.backup.*` it leaves behind when you go the other way.
  Or keep each config in its own directory and pass
  `setup_path:=/path/to/dir/` to the launch.
- **Scoring cost grows with run length** — a distance matrix of samples x 4979
  centreline points, ~120 MB for a 340 s run.
- **Cross-track error is measured to the nearest centreline sample**, which are
  3.2 cm apart, so reported error is conservative by up to ~1.6 cm.

## Troubleshooting

**`import cv2` fails with `numpy.core.multiarray failed to import`** — a pip
numpy in `~/.local` is shadowing the apt one that `python3-opencv` is built
against. Either remove it (`pip uninstall --break-system-packages numpy`) or
install a numpy-2-compatible OpenCV (`pip install --user --no-deps
--break-system-packages opencv-python-headless`).

**The robot ignores `cmd_vel`** — check what the controller_manager is running:

```bash
ros2 service call /a200_0000/controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers
```

(`ros2 control list_controllers -c /a200_0000/controller_manager` says the same
thing, but it needs `ros-$ROS_DISTRO-ros2controlcli`, which
`ros-$ROS_DISTRO-controller-manager` does not pull in.)

Whether Clearpath spawns these itself depends on the release.
`clearpath-simulator` 2.9.3 and earlier spawned neither of the a200's, because
`control.launch.py`'s sim-mode loop only took controllers whose name contains
`controller` and contains neither `manager` nor `platform` — and
`joint_state_broadcaster` and `platform_velocity_controller` each fail that
filter, so `controller_manager` came up empty. 2.9.4 spawns both correctly.

`line_follow_sim.launch.py` therefore does not spawn them blindly — a second
spawner against an already-active controller exits 1 with *"can not be
configured from 'active' state"*. It runs `scripts/ensure_controllers.py` on a
10 s timer instead, which asks `controller_manager` what is already active and
spawns only what is missing. That is the place to look if they never appear.

**`gz sim` segfaults on startup** — check `nvidia-smi` for a driver/kernel
module mismatch before suspecting these packages. Gazebo needs a working
GL/EGL context for the camera even when headless.
