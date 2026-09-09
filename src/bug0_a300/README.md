# bug0_a300

Bug0 navigation controller for the Clearpath A300 Observer, ported from
`bug0_turtlebot4`. Same algorithm; only the platform bits changed:

| | TurtleBot4 | A300 Observer |
|---|---|---|
| namespace | none | `a300_00000` (`/tf`, `/tf_static` remapped into it) |
| range sensor | 2D `/scan` | 3D `sensors/lidar3d_1/points`, flattened to `sensors/lidar3d_1/scan_2d` (LaserScan in `base_link`) by `pointcloud_to_laserscan` |
| lidar yaw offset | 90 deg | 0 deg (scan already in `base_link`) |
| cmd_vel | `/cmd_vel` | `<ns>/cmd_vel` (TwistStamped) |
| distances / speeds | 0.17 m robot | scaled for the ~0.99 x 0.67 m chassis (see constants in `bug0_node.py`) |

Pose comes from TF `map -> base_link`, exactly as before. In the
`husky_bamboo_sim` A300 simulation (`a300_sim.sh`) `map -> odom` comes from Nav2
AMCL on the 2D lidar, seeded at the spawn pose (`localization:=true`, the default), and
the generated maps share Gazebo's world frame, so goals are given in world coordinates.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select bug0_a300 --symlink-install
source install/setup.bash
```

## Run (simulation)

```bash
# terminal 1: headless Gazebo + RViz (office world)
~/ros2_ws/scripts/a300_sim.sh office

# terminal 2
ros2 launch bug0_a300 bringup.launch.py goal_x:=-7.0 goal_y:=-1.0 wall_side:=auto record_bag:=false
```

Running the node by itself:

```bash
ros2 run bug0_a300 bug0_node --ros-args -r __ns:=/a300_00000 \
  -r /tf:=/a300_00000/tf -r /tf_static:=/a300_00000/tf_static \
  -p use_sim_time:=true -p goal_x:=-7.0 -p goal_y:=-1.0
```

(the flattened scan must already be published; the launch file starts it).

## Notes on the 3D lidar

The Ouster sits on the sensor arch ~0.85 m above the floor with a +/-15 deg vertical
field of view and a 0.9 m minimum range. Points between `scan_min_height` and
`scan_max_height` (in `base_link`) are kept, which drops floor returns and the robot's
own body. Objects lower than roughly 0.5 m are outside the vertical FOV when they are
within ~1 m of the robot, so walls, shelves and furniture are seen but very low
obstacles right at the bumper are not.
