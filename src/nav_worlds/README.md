# nav_worlds

Point-to-point navigation with SLAM and Nav2 for the Clearpath **a200** and the
Unitree **go1**, across the Gazebo worlds shipped with `clearpath_gz` plus
`depot`.

The navigation itself is Clearpath's: `clearpath_nav2_demos` provides the
`slam`, `localization` and `nav2` launches and the per-platform parameter files.
This package supplies the parts that were missing to make those work here.

## Run

```bash
# Point-to-point goal navigation for A200 with SLAM (builds a map as it drives):
ros2 launch nav_worlds a200_point_nav.launch.py world:=warehouse

# With AMCL localization against a saved map:
ros2 launch nav_worlds a200_point_nav.launch.py world:=warehouse slam:=false

# With Gazebo GUI enabled:
ros2 launch nav_worlds a200_point_nav.launch.py world:=warehouse headless:=false
```

*(Note: `ros2 run nav_worlds bringup.sh [world]` forwards to `a200_point_nav.launch.py`)*

Then send goals, in the `map` frame:

```bash
ros2 run nav_worlds send_goal.py 4.0 0.0          # x, y
ros2 run nav_worlds send_goal.py 4.0 2.0 90       # x, y, yaw in degrees
```

`send_goal.py` reports SUCCEEDED/FAILED with the time taken and how close the
robot actually got, so a run can be scored rather than eyeballed.

### Automated Simulation Sweeps (Dataset Collection)

To collect reproducible datasets (ROS 2 MCAP bags and synchronized `.jsonl` telemetry logs) over batches of independent trajectories:

```bash
# 1. Sample reachable, collision-free waypoints from a static map:
ros2 run nav_worlds generate_waypoints.py \
  --map-yaml $(ros2 pkg prefix clearpath_nav2_demos)/share/clearpath_nav2_demos/maps/warehouse.yaml \
  --seed 42 -n 10 -o data/warehouse_waypoints.csv

# 2. Run automated headless simulation sweep:
ros2 run nav_worlds run_sweep.py \
  --waypoints data/warehouse_waypoints.csv \
  --max_runs 3

# 3. Visual Debugging Mode (Gazebo GUI and RViz):
# (Requires unsandboxed execution per AGENTS.md for host X11 display socket access)
ros2 run nav_worlds run_sweep.py \
  --waypoints data/warehouse_waypoints.csv \
  --gui \
  --rviz
```

Outputs are structured under `data/dataset_output/sweep_YYYYMMDD_HHMMSS/` containing `sweep_metadata.csv`, and per-run folders with `state.jsonl` (time, pose, twist, full 720-beam scan) and `bag/`.

Worlds: `warehouse`, `office`, `construction`, `solar_farm`, `orchard`,
`pipeline` come from `clearpath_gz`; `depot` is carried here.

## Why this package exists

Five things stop the stock pieces from working together. Each fails silently or
misleadingly, so they are worth stating.

**The lidar sees the robot, and Nav2 believes it.** With the sensor arch fitted,
the arch legs sit about 0.40 m from the 2D lidar, inside the robot's 1.1 x 0.9 m
footprint. 32 of 720 beams return the robot's own structure, and Nav2's
`collision_monitor` (`FootprintApproach`, `min_points: 12`) reads them as a
permanently imminent collision, scaling every command to zero. The robot accepts
goals, plans paths, reports "Passing new path to controller", and never moves.

`scripts/scan_self_filter.py` discards returns that land inside the footprint and
republishes on `sensors/lidar2d_0/scan_filtered`; `bringup.sh` points SLAM and
Nav2 at that topic via their `scan_topic` argument. The closest return goes from
0.401 m to 1.099 m, and the 683 beams that see the world are passed through
byte-identical.

Testing the *endpoint* matters. The lidar is mounted at x=0.328 m, which is
already inside the footprint, so masking whole bearings that the footprint covers
masks the entire scan - all 720 beams. Nothing useful is lost by the endpoint
rule: a return inside the footprint is either the robot or something the robot is
already touching, and projecting either forward in time to predict a collision is
meaningless. It is the same rule the Nav2 costmap applies as
`footprint_clearing_enabled`.

Deleting the arch also fixes it, and an earlier version of this package shipped a
config that did. That is the wrong trade: the arch carries the camera, and the
real robot has one.

**Nothing may start until the simulation is at speed.** A heavy world sits near
RTF 0.0005 while Gazebo loads meshes. Anything started in that window fails in
its own way and none of the messages point at the cause:

| Started too early | How it fails |
| --- | --- |
| controller spawner | `Switch controller timed out after 5 seconds`, spawner dies, robot uncontrollable |
| slam_toolbox | logs `Configuring`, never reaches `Activating`, no `/map` ever |
| nav2 | `failed to send response to change_state (timeout)`, servers strand unconfigured |

`bringup.sh` waits for the world, the robot, a settled RTF, the lidar, active
controllers, a `/map`, and finally the `navigate_to_pose` action - each on
observable state rather than a fixed delay, because load times differ by an
order of magnitude between worlds.

**`clearpath_gz`'s `simulation.launch.py` only accepts its own six worlds.** Its
`world` argument carries a `choices` list, so `depot` - or any path - is
rejected before Gazebo is reached. `launch/sim.launch.py` composes the same
`gz_sim` + `robot_spawn` underneath without that restriction. Note that the two
need different things from `world`: `gz_sim` wants a file, `robot_spawn` wants
the name inside the SDF, for its `/world/<name>/create` service.

**There is no way to run `clearpath_gz` headless.** It builds the `gz_args`
string itself and exposes no hook, so `-s --headless-rendering` cannot be passed
through. `sim.launch.py` takes `headless:=true` (the default from `bringup.sh`)
and calls `ros_gz_sim` directly, carrying over the three things `clearpath_gz`
sets up that are not optional: the resource path, the clock bridge, and the
world. `--headless-rendering` is needed *in addition to* `-s`, or rendering
sensors publish nothing - and that failure is silent, with the camera topic
advertised and simply never delivering.

**Every shipped copy of the depot world has the Sensors system commented out.**
Without it the 2D lidar publishes nothing at all, so SLAM builds no map and Nav2
has nothing to plan on. `worlds/depot.sdf` enables `Sensors` and adds `Imu`,
which the EKF needs for `odom -> base_link`.

**The default global costmap rejects goals outside a 10-meter radius.**
Clearpath's default `a200/nav2.yaml` sets `global_costmap.rolling_window: true`
with dimensions $20\text{ m} \times 20\text{ m}$. Because the rolling window is
centered on the robot, any goal exceeding $10\text{ m}$ Euclidean distance from
the start pose is instantly rejected with `"Goal Coordinates was outside bounds"`.
Rather than mutating the default demo configuration, `config/nav2_static.yaml`
provides a dedicated duplicate with `rolling_window: false`. When navigating
against static maps (`slam:=false`), `a200_point_nav.launch.py` automatically
routes to `nav2_static.yaml`, expanding the global costmap to the full map extent.

## Layout

| Path | Description |
| --- | --- |
| `launch/a200_point_nav.launch.py` | Native ROS 2 point-to-point navigation bringup (sim + SLAM/AMCL + Nav2). |
| `scripts/run_sweep.py` | Automated data collection orchestrator (warm reset default, optional cold restart, rosbag + JSONL logging). |
| `scripts/generate_waypoints.py` | Deterministic free-space waypoint sampler with obstacle clearance and reachability checks. |
| `scripts/log_state.py` | Synchronized telemetry logger (pose, twist, 720-beam scan array) dumping to JSON Lines. |
| `scripts/readiness_gate.py` | Readiness gate ensuring simulation & controllers are active before Nav2 starts. |
| `scripts/bringup.sh` | Backward-compatibility forwarder to `a200_point_nav.launch.py`. |
| `scripts/send_goal.py` | Send a goal and report the outcome. |
| `launch/sim.launch.py` | World + robot, without the `choices` restriction. |
| `launch/gz_sim.launch.py` | Gazebo simulator launch with `shell=False` for direct signal propagation and clean GUI teardown. |
| `launch/husky_nav.launch.py` | Legacy single-shot launch (uses fixed delays). |
| `scripts/scan_self_filter.py` | Drops laser returns landing inside the footprint. |
| `config/nav2_static.yaml` | Static-map Nav2 configuration with global costmap rolling window disabled. |
| `config/a200_sample.yaml` | a200 config used for navigation and teleop: sensor arch, realsense, 2D and 3D lidar. Copy to `~/clearpath/robot.yaml`. |
| `worlds/depot.sdf` | Depot with the Sensors and Imu systems enabled. |
| `maps/` | Saved maps for localization mode. |
| `doc/SIMULATION_SWEEP.md` | Detailed architectural design, failure modes, and decisions for the sweep pipeline. |
