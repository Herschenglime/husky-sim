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
