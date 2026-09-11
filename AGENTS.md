# Agent Guidelines

Operational constraints and execution rules for autonomous agents working in this repository.

---

## 1. Gazebo GUI Execution (`headless:=false`)
* **Finding:** Launching Gazebo Harmonic with the GUI enabled (`headless:=false`) fails inside the standard sandbox due to host display socket (`$DISPLAY`, `/tmp/.X11-unix`) and graphics isolation.
* **Rule:** Always execute commands launching Gazebo with `headless:=false` using **`BypassSandbox: true`**. Headless runs (`headless:=true`) can remain in the standard sandbox.

---

## 2. Trajectory Sweep Scope
* **Rule:** Navigation sweeps and validation runs must focus strictly on **independent, one-way trajectories with unique start and end points**.
* Do not batch waypoints into continuous multi-waypoint patrol trajectories (`goThroughPoses` or `followWaypoints`) unless explicitly requested by the user.

---

## 3. Teardown & Process Cleanup
* **Finding:** Terminating Gazebo Harmonic via standard `killall gz-sim-server` fails because the command line is `gz sim server` / `gz sim gui`. Orphaned simulations continue publishing to `/clock` and sensor topics, corrupting sim time and blocking Nav2. In addition, lingering `parameter_bridge`, `image_bridge`, `static_transform_publisher`, and ROS 2 daemon discovery caches pollute the TF tree and costmap buffers across simulation runs.
* **Rule:** Always execute a full cold teardown to ground zero using the dedicated cleanup script:
  ```bash
  ./cold_restart.sh
  ```
* **Runner Self-Termination Caution:** When running teardown from inside an automated runner or agent script launched via `ros2 run`, do NOT run an unfiltered `pkill -9 -f "ros2"`, as it will kill the runner and parent shell. Use PID/PPID filtering (e.g., as implemented in `run_sweep.py:cleanup_orphans`).

---

## 4. Simulation Time Synchronization (`use_sim_time`)
* **Finding:** Nodes or CLI scripts publishing goals (such as `send_goal.py`) without `use_sim_time:=true` stamp messages with system wall time (e.g. year 2026), while Gazebo publishes `/clock` starting from 0. Nav2 rejects these goals or drops transforms.
* **Rule:** Always pass `--use-sim-time` (or `-p use_sim_time:=true`) when sending goals or running standalone nodes against a simulation.

---

## 5. Non-Interactive CLI Automation
* **Finding:** `collect_dataset.py` launches `xdg-open` on the waypoint preview image and prompts interactive confirmation on stdin (`[Y/n]`) by default. In headless or agent environments, this blocks indefinitely.
* **Rule:** Always pass `-y` / `--no-prompt` when calling `collect_dataset.py` in scripts or automated workflows.

