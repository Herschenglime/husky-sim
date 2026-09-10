# Agent Handoff Document: Automated Simulation Data Collection

This document provides a comprehensive summary of completed work, key operational findings, and actionable next steps for continuing development on the automated data collection pipeline in `ros2_ws`.

---

## 1. Project Context & Objectives

* **Workspace:** `/home/pgrau/ros2_ws`
* **Stack:** ROS 2 Jazzy, Gazebo Harmonic, Nav2, Clearpath Husky A200 (`a200_0000` namespace)
* **Goal:** Implement an autonomous data collection pipeline to sample valid start/goal waypoints from a 2D occupancy map, spawn Gazebo simulations headlessly per trajectory, record lightweight synchronized state logs (JSONL with 2D Lidar) and ROS bags, and manage clean process lifecycle across multi-run sweeps.

---

## 2. Task Checklist Status (`task.md` Structure)

- [x] **1. Implement `generate_waypoints.py`**
  - [x] Write script to parse `.yaml` and `.pgm` map files.
  - [x] Sample free-space points with obstacle clearance and reachability guarantees.
  - [x] Output pairs to `waypoints.csv`.
  - [x] **Verification**: Generated `data/warehouse_waypoints.csv` from `warehouse.yaml` and verified coordinates.

- [x] **2. Verify Waypoints in Simulation**
  - [x] Sample an independent one-way trajectory from `data/warehouse_waypoints.csv`.
  - [x] Launch `a200_point_nav.launch.py` specifying `x`, `y`, `yaw` start coordinates.
  - [x] Dispatch goal coordinates using `send_goal.py`.
  - [x] **Verification**: Verified robot spawn, localization, path planning, and arrival at the goal.

- [x] **3. Implement `log_state.py`**
  - [x] Write script subscribing to odometry, velocity, and 2D lidar topics.
  - [x] Dump state data to a lightweight, human-readable JSON Lines (`.jsonl`) file.
  - [x] **Verification**: Executed live trial with AMCL; verified 1,189 monotonically increasing samples logged with complete 720-beam scan arrays and clean process teardown.

- [x] **4. Implement `run_sweep.py` Orchestrator**
  - [x] Write subprocess management logic (launch simulation, `ros2 bag record`, `log_state.py`).
  - [x] Write goal dispatch and timeout/success detection logic.
  - [x] Write dataset organization logic (timestamped sweep folders, `run_XXX/` subdirectories, metadata CSV).
  - [x] Maintain initial Real-Time Factor (RTF) near 1.0 for stability.
  - [x] Support visual debugging options (`--gui` and `--rviz`).

- [x] **5. End-to-End Test & Verification**
  - [x] Verify Gazebo teardown and bringup works cleanly between iterations without stranded processes.
  - [x] Verify `cmd_vel` recorded cleanly without DDS graph type conflicts (`TwistStamped`).
  - [x] Verify full long-distance static warehouse navigation with `nav2_static.yaml`.
  - [x] Inspect output dataset: `state.jsonl` (3,407 frames, 46.7 MB), `bag/` (46,166 messages), and `sweep_metadata.csv`.

---

## 3. Completed Work & Key Implementations

### Task 1: Waypoint Generation (`src/nav_worlds/scripts/generate_waypoints.py`)
* Parses map metadata (`.yaml`) and occupancy raster (`.pgm`).
* Uses morphological inflation to maintain obstacle clearance.
* Samples $N$ pairs of `(start_x, start_y, start_yaw_rad, start_yaw_deg, goal_x, goal_y, goal_yaw_deg, distance)`.
* Enforces minimum Euclidean separation between start and goal.
* Executable script located at `src/nav_worlds/scripts/generate_waypoints.py`; verified output at `data/warehouse_waypoints.csv`.

### Task 2: Simulation Verification & Nav2 Integration (`src/nav_worlds/scripts/send_goal.py`)
* Refactored `send_goal.py` to use `nav2_simple_commander.robot_navigator.BasicNavigator`.
* Added `navigator.waitUntilNav2Active(localizer='robot_localization')` to eliminate race conditions where goals were rejected by un-activated lifecycle nodes (`bt_navigator`).
* Added automatic visual sphere marker spawning in Gazebo at goal coordinates.

### Task 3: State & Lidar Logger (`src/nav_worlds/scripts/log_state.py`)
* **Format:** Chose **JSON Lines (`.jsonl`)** over CSV to natively support the variable/array data from the 2D Lidar sweep.
* **Topics subscribed:**
  * Odom: `/a200_0000/platform/odom/filtered`
  * Cmd Vel: `/a200_0000/cmd_vel` (`geometry_msgs/msg/TwistStamped`)
  * Scan: `/a200_0000/sensors/lidar2d_0/scan_filtered`
* **Log Schema:** `{"timestamp": <float>, "x": <float>, "y": <float>, "yaw": <float>, "vx": <float>, "wz": <float>, "scan": [<720 floats / nulls>]}`.
* Driven by the `LaserScan` callback to log each sweep with the latest odometry and twist state without duplicating large arrays. Safely maps `inf` and `nan` to `null`.
* Uses `self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])` for sim time alignment in ROS 2 Jazzy.

### Task 4: Sweep Orchestrator (`src/nav_worlds/scripts/run_sweep.py`)
* Implemented modular runner architecture:
  * `SimulationRunner`: Base class handling waypoint parsing, directory layout, and metadata tracking.
  * `ColdRestartRunner`: Subclass implementing cold restarts (Gazebo spawn, rosbag recording, telemetry capture, and clean teardown per trajectory). Prepares the interface cleanly for future in-process "Warm Resets".
* Supported CLI flags:
  * `--waypoints`: Path to waypoints CSV.
  * `--output_dir`: Output root (default `data/dataset_output`).
  * `--max_runs`: Cap on trajectories executed.
  * `--gui`: Spawns Gazebo GUI (`headless:=false`) for visual debugging.
  * `--rviz`: Launches RViz visualization (`rviz:=true`).
  * `--include-camera`: Optionally records camera topics in rosbag.
  * `--disable-lidar2d`: Optionally omits 2D lidar from rosbag.
* **Teardown & Lifecycle Protection**: Selective cleanup logic kills orphaned `gz sim` processes and dangling `ros2` nodes while explicitly protecting the runner's own PID and parent process.

### Task 5: Static Nav2 Configuration (`src/nav_worlds/config/nav2_static.yaml`)
* Duplicated `clearpath_nav2_demos/config/a200/nav2.yaml` into `nav_worlds` with `global_costmap.rolling_window: false`.
* Updated `clearpath_nav2_demos/launch/nav2.launch.py` to accept `nav2_yaml` parameter without breaking default behavior.
* Updated `a200_point_nav.launch.py` to transparently route to `nav2_static.yaml` when `slam:=false`.

---

## 4. Critical Discoveries & Operational Rules

### 1. Process Teardown Rule (`AGENTS.md`)
> **Never use standard `killall gz-sim-server`!**
> In Gazebo Harmonic, the processes are invoked as `gz sim server` and `gz sim gui`. Using `killall gz-sim-server` fails silently, leaving orphaned simulation servers running.
> If an orphaned server survives, multiple simulation clocks will publish to `/clock` simultaneously. This causes timestamps to oscillate wildly (e.g. jumping back and forth by 200+ seconds), which triggers Nav2's `collision_monitor` and AMCL message filters to reject all sensor data.
> **Rule:** Always terminate simulation processes using:
> ```bash
> pkill -9 -f "gz sim" && pkill -9 -f "ros2"
> ```

### 2. Gazebo GUI Sandbox Constraint (`AGENTS.md`)
> Launching Gazebo with GUI enabled (`headless:=false`) inside the standard sandbox fails due to host X11 display socket (`/tmp/.X11-unix`, `$DISPLAY`) and GPU isolation.
> **Rule:** Always execute commands launching Gazebo with `headless:=false` using **`BypassSandbox: true`**. Headless runs (`headless:=true`) can remain in the standard sandbox.

### 3. One-Way Trajectory Constraint (`AGENTS.md`)
> Navigation sweeps must focus strictly on **independent, one-way trajectories with unique start and end points** using `navigator.goToPose()`. Do not batch waypoints into multi-waypoint patrol trajectories (`goThroughPoses`/`followWaypoints`).

### 4. AMCL Initialization Seeding in `readiness_gate.py`
* In `slam:=false` mode, AMCL must receive an initial pose before it publishes the `map -> odom` transform.
* Updated `src/nav_worlds/scripts/readiness_gate.py` (`_run_map_stage`) to publish to `/{namespace}/initialpose` with `TRANSIENT_LOCAL` durability and loop until `tf_buffer.can_transform('map', 'odom', ...)` succeeds.

### 5. Automatic Launch Teardown on Goal Arrival
* Added `on_exit=Shutdown()` to the `send_goal` Node in `src/nav_worlds/launch/a200_point_nav.launch.py`. When `run_goal:=true` is used, the entire launch stack (Gazebo, Nav2, and bridges) now terminates cleanly once the goal is reached.

### 6. `TwistStamped` Message Alignment in ROS 2 Jazzy
* In Jazzy, Clearpath robot platforms publish `geometry_msgs/msg/TwistStamped` on `/{ns}/cmd_vel`.
* Subscribing with legacy `Twist` caused a dual-type DDS collision: `ros2 bag record` emitted `Topic has more than one type associated. Skipping.` and refused to capture commands.
* Aligned `log_state.py` to `TwistStamped` by default with an optional `--unstamped_cmd_vel` fallback.

### 7. Global Costmap Rolling Window Bounds Failure
* The stock Clearpath `a200/nav2.yaml` sets `global_costmap.rolling_window: true` with a $20\text{ m} \times 20\text{ m}$ boundary ($\pm 10\text{ m}$ reach from spawn).
* Any goal $>10\text{ m}$ away is instantly aborted by NavfnPlanner with `"Goal Coordinates was outside bounds"`.
* Resolved by creating `src/nav_worlds/config/nav2_static.yaml` (`rolling_window: false`) for static navigation, leaving the shared demo config untouched.

### 8. Runner Self-Termination Avoidance
* Invoking `pkill -9 -f "ros2"` inside `run_sweep.py` killed the sweep runner itself when launched via `ros2 run nav_worlds run_sweep.py`.
* `cleanup_orphans()` explicitly protects `os.getpid()` and `os.getppid()`, terminating only child/external simulation nodes.

### 9. Gazebo GUI Clean Shutdown via Direct Process Execution (`gz_sim.launch.py`)
* **Issue**: On programmatic `Shutdown()` (such as goal arrival triggering `OnProcessExit -> Shutdown()`), the Gazebo GUI window lingered on screen while all other ROS 2 nodes terminated cleanly.
* **Root Cause**: Upstream `ros_gz_sim`'s `gz_sim.launch.py` launches Gazebo through `/bin/sh -c "..."` (`shell=True`). On programmatic `Shutdown()`, ROS 2 launch sends `SIGINT` only to `/bin/sh` (which ignores it in non-interactive mode while waiting on children), and then escalates to `SIGKILL` on `/bin/sh`. The child `ruby` wrapper and detached `gz sim gui` process group were orphaned and never received a termination signal.
* **Resolution**: Created `src/nav_worlds/launch/gz_sim.launch.py` as a customized duplicate of upstream `ros_gz_sim/launch/gz_sim.launch.py`:
  - Retains all upstream environment variables (`GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_SIM_RESOURCE_PATH`), package export scanning (`GazeboRosPaths`), and launch arguments (`--force-version`, `debugger`, `on_exit_shutdown`).
  - Replaces `shell=True` with `shell=False` using `shlex.split` for command tokens.
  - Updated `src/nav_worlds/launch/sim.launch.py` to delegate to `nav_worlds/launch/gz_sim.launch.py`.
  - Signal delivery now flows directly from ROS 2 launch to `ruby`, activating its `Signal.trap("INT")` handler to cleanly kill both `gz sim gui` and `gz sim server` upon shutdown. Verified complete GUI and simulation teardown in 36.48s.

### 10. Warm Resets (`WarmRestartRunner`)
* **Implementation**: Introduced `WarmRestartRunner` in `src/nav_worlds/scripts/run_sweep.py` selectable with `--warm-reset`.
* **Mechanism**:
  - Launches Gazebo and Nav2 stack once at the start with `run_goal:=false`.
  - Between runs: zeros velocity (`TwistStamped`), teleports `a200_0000/robot` via Gazebo service `/world/{world}/set_pose`, publishes AMCL seed pose to `/{ns}/initialpose`, and clears local & global costmaps via `nav2_msgs/srv/ClearEntireCostmap`.
  - Per-run logging: Spawns fresh `log_state.py` and `ros2 bag record` instances per trajectory for complete data isolation.
  - Eliminates the 25–30s simulator bootup penalty between trajectories, reducing test trajectory turnaround to ~15s.

---

## 5. Next Steps for Incoming Agent

1. **Large-Scale Warm Reset Sweeps**:
   - Execute full sweeps across generated batches (e.g. 10 to 50 trajectories) in the warehouse and depot worlds using `--warm-reset`:
     ```bash
     python3 src/nav_worlds/scripts/run_sweep.py --waypoints data/warehouse_waypoints.csv --warm-reset
     ```
2. **Multi-World Waypoint Generation**:
   - Run `generate_waypoints.py` against other worlds (`depot`, `office`, `construction`) to build multi-environment training corpora.
3. **Dataset Ingestion Pipeline**:
   - Verify downstream training format compatibility with `state.jsonl` (timestamps monotonically increase across warm resets; apply $t - t_0$ zero-offsetting if needed).
