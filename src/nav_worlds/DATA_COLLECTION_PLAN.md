# Automated Simulation Data Collection

We will implement a Python-based orchestrator to repeatedly sample start/end coordinates from a static map, spawn the Gazebo simulation headlessly at the start coordinates, record ROS topics and a lightweight CSV state log, and drive the robot to the goal using Nav2.

## Web Search Findings

I performed a web search for existing ROS 2 Nav2 simulation data collection tools. While the Nav2 stack provides the **Simple Commander API** for scripting waypoints and there are benchmarking tools in `navigation2/tools` (like `opennav_robotics_workload_benchmark`), there is no widely-adopted, drop-in tool that handles the full lifecycle we need: automatically parsing free-space from a map, handling Gazebo headless startup/teardown per run, and organizing dataset folders. The community standard is exactly what we are planning: building a custom Python orchestrator that utilizes `subprocess`, `ros2 bag`, and `rclpy` action clients.


## Proposed Changes

### 1. Waypoint Generation (Extensible)

We will create a script that reads any given static map, identifies free space, and randomly selects valid start and end coordinates.

#### [NEW] [generate_waypoints.py](file:///home/pgrau/ros2_ws/src/nav_worlds/scripts/generate_waypoints.py)
- **Extensibility**: Accepts `--map-yaml` as an argument to support any map (defaults to `warehouse.yaml`).
- Loads the YAML, parses the corresponding `.pgm` image.
- Finds all free pixels (value > free_thresh).
- Converts pixel coordinates to real-world coordinates `(x, y)` using the map's origin and resolution.
- Randomly samples $N$ pairs of `(start_x, start_y, end_x, end_y)`.
- Enforces a minimum Euclidean distance between start and end points.
- Outputs to `waypoints.csv`.

### 2. Data Logging

We created a lightweight node to log state variables and 2D LiDAR scans directly to JSON Lines (`.jsonl`) for quick ML iteration without parsing rosbags.

#### [NEW] [log_state.py](file:///home/pgrau/ros2_ws/src/nav_worlds/scripts/log_state.py)
- **Extensibility**: Accepts `--odom_topic`, `--cmd_vel_topic`, and `--scan_topic` arguments to support different robots (e.g. `/a200_0000` or `/a300_0000`).
- **TwistStamped Support**: Subscribes to `geometry_msgs/msg/TwistStamped` by default to avoid DDS multi-type conflicts in ROS 2 Jazzy, with an optional `--unstamped_cmd_vel` flag for legacy robots.
- Writes a `.jsonl` file containing: `timestamp`, `x`, `y`, `yaw`, `vx`, `wz`, and the full 720-beam `scan` array.
- Driven by the `LaserScan` callback to log synchronized snapshots whenever new LiDAR packets arrive.
- Gracefully closes output on `SIGINT`.

### 3. Sweep Orchestration & Data Organization

We created the main orchestrator script that loops through the generated waypoints, drives the simulation lifecycle, and organizes the output data.

#### [NEW] [collect_dataset.py](file:///home/pgrau/ros2_ws/src/nav_worlds/scripts/collect_dataset.py)
- **Unified Pipeline**: Wraps waypoint generation, interactive trajectory inspection (`xdg-open`), and automated sweep execution in a single command. Use `-y` for non-interactive execution.

#### [NEW] [run_sweep.py](file:///home/pgrau/ros2_ws/src/nav_worlds/scripts/run_sweep.py)
- **Real-Time Factor (RTF)**: Gazebo physics cap is maintained near real-time (RTF ~ 1.0) for initial stability.
- **Modular Runner Architecture**: Structured with a base class `SimulationRunner` (managing waypoints, output directories, and sweep metadata), `WarmRestartRunner` (default in-process teleportation), and `ColdRestartRunner` (managing cold restart bringup, bag recording, and process teardown per run).
- **Static Nav2 Costmap Support**: Routes navigation to `src/nav_worlds/config/nav2_static.yaml` (`rolling_window: false`) in localization mode to avoid out-of-bounds trajectory aborts on distant waypoints (> 10m).
- **Visual Debugging**: Accepts `--gui` (Gazebo GUI via `headless:=false`) and `--rviz` (RViz visualization) for interactive debugging.
- **Data Organization**: Creates a timestamped parent directory for the entire sweep under `data/dataset_output/`:
  ```text
  data/dataset_output/
    sweep_YYYYMMDD_HHMMSS/
      sweep_metadata.csv       # Summary of each trajectory run (run_id, start, goal, status, elapsed_time)
      run_000/
        bag/                   # The MCAP rosbag directory (odom, tf, tf_static, cmd_vel, scan_filtered, plan, clock)
        state.jsonl            # Synchronized telemetry (pose, twist, 720-beam scan array)
      run_001/
        ...
  ```
- **Lifecycle Management**: For each pair in the waypoint list:
  1. Executes orphan cleanup (`cleanup_orphans()`) terminating lingering `gz sim` and ROS 2 nodes while preserving runner PIDs.
  2. Spawns `ros2 bag record` to capture minimal topics into `run_XXX/bag/`.
  3. Spawns `log_state.py` to capture `run_XXX/state.jsonl`.
  4. Launches `a200_point_nav.launch.py` with the trajectory's start and goal coordinates (`run_goal:=true`, `slam:=false`).
  5. Waits for the launch process to exit on goal arrival (or triggers timeout if stuck).
  6. Sends `SIGTERM` / `SIGKILL` to bagger and logger processes.
  7. Pauses briefly (2.0s) to ensure sockets and ports are freed before the next iteration.

## Verification Plan

### Manual Verification
- Generate a batch of 3 waypoints using `generate_waypoints.py` using the warehouse map.
- Run `run_sweep.py` on the 3 waypoints.
- Verify that Gazebo spawns, Nav2 plans a path, the robot moves, and upon completion, the simulation is torn down and restarts with the next waypoint.
- Verify the output directory structure matches the `sweep_YYYYMMDD_HHMMSS` design.
- Verify the bag files contain the necessary reproducible topics (and omit large camera data).
