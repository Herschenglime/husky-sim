# nav_worlds

Point-to-point navigation with SLAM and Nav2 for the Clearpath **a200** and the
Unitree **go1**, across the Gazebo worlds shipped with `clearpath_gz` plus
`depot`.

The navigation itself is Clearpath's: `clearpath_nav2_demos` provides the
`slam`, `localization` and `nav2` launches and the per-platform parameter files.
This package supplies the parts that were missing to make those work here.

## Run

```bash
ros2 run nav_worlds bringup.sh warehouse          # SLAM, builds a map as it drives
ros2 run nav_worlds bringup.sh warehouse --localize   # AMCL against maps/warehouse.yaml
```

Then send goals, in the `map` frame:

```bash
ros2 run nav_worlds send_goal.py 4.0 0.0          # x, y
ros2 run nav_worlds send_goal.py 4.0 2.0 90       # x, y, yaw in degrees
```

`send_goal.py` reports SUCCEEDED/FAILED with the time taken and how close the
robot actually got, so a run can be scored rather than eyeballed.

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

## Layout

| Path | Description |
| --- | --- |
| `scripts/bringup.sh` | Staged bringup: world, robot, SLAM or AMCL, Nav2. |
| `scripts/send_goal.py` | Send a goal and report the outcome. |
| `launch/sim.launch.py` | World + robot, without the `choices` restriction. |
| `launch/husky_nav.launch.py` | Single-shot launch. Convenient, but uses fixed delays - prefer `bringup.sh` on heavy worlds. |
| `scripts/scan_self_filter.py` | Drops laser returns landing inside the footprint. |
| `config/a200_sample.yaml` | a200 config used for navigation and teleop: sensor arch, realsense, 2D and 3D lidar. Copy to `~/clearpath/robot.yaml`. |
| `worlds/depot.sdf` | Depot with the Sensors and Imu systems enabled. |
| `maps/` | Saved maps for `--localize`. |
