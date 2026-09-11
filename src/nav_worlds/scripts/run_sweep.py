#!/usr/bin/env python3
import os
import sys
import csv
import time
import math
import shutil
import argparse
import subprocess
import threading
from datetime import datetime

try:
    import rclpy
    from geometry_msgs.msg import (
        PoseStamped,
        PoseWithCovarianceStamped,
        Twist,
        TwistStamped,
    )
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ClearEntireCostmap, SetInitialPose
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from std_srvs.srv import Empty

    sys.path.insert(0, os.path.dirname(__file__))
    from send_goal import publish_goal_marker
    from bag_recorder import RosbagRecorder
except ImportError:
    rclpy = None

def resolve_bag_topics(args, ns):
    if getattr(args, 'custom_topics', None):
        return args.custom_topics

    profile = getattr(args, 'bag_profile', 'standard') or 'standard'
    profile = profile.lower()

    # Minimal profile: odom, cmd_vel, tf, tf_static, clock
    minimal = [
        f'/{ns}/platform/odom/filtered',
        f'/{ns}/cmd_vel',
        f'/{ns}/tf',
        f'/{ns}/tf_static',
        '/clock'
    ]

    # Standard profile (default): minimal + 2d lidar, plan, goal_pose
    standard = list(minimal) + [
        f'/{ns}/plan',
        f'/{ns}/goal_pose'
    ]
    if not getattr(args, 'disable_lidar2d', False):
        standard.append(f'/{ns}/sensors/lidar2d_0/scan_filtered')

    # Perception profile: standard + camera
    perception = list(standard) + [
        f'/{ns}/sensors/camera_0/color/image',
        f'/{ns}/sensors/camera_0/color/camera_info'
    ]

    # Full profile: perception + 3d lidar + joint states
    full = list(perception) + [
        f'/{ns}/sensors/lidar3d_0/velodyne_points',
        f'/{ns}/joint_states'
    ]

    if profile == 'minimal':
        topics = list(minimal)
    elif profile == 'perception':
        topics = list(perception)
    elif profile == 'full':
        topics = list(full)
    else:  # 'standard'
        topics = list(standard)

    # Legacy flags overrides if profile wasn't explicitly set to perception/full
    if getattr(args, 'include_camera', False) and profile not in ('perception', 'full'):
        for c in [f'/{ns}/sensors/camera_0/color/image', f'/{ns}/sensors/camera_0/color/camera_info']:
            if c not in topics:
                topics.append(c)

    if getattr(args, 'add_topics', None):
        for t in args.add_topics:
            topic = t if t.startswith('/') else f'/{ns}/{t}'
            if topic not in topics:
                topics.append(topic)

    return topics


class SimulationRunner:
    def __init__(self, args):
        self.args = args
        self.ns = "a200_0000"
        
        # Setup output directory
        if getattr(self.args, 'sweep_dir', None):
            self.sweep_dir = self.args.sweep_dir
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.sweep_dir = os.path.join(self.args.output_dir, f"sweep_{timestamp}")

        if os.path.exists(self.sweep_dir):
            existing = set(os.listdir(self.sweep_dir))
            existing -= {'waypoints.csv', 'waypoints_preview.png'}
            if existing:
                if not getattr(self.args, 'overwrite', False):
                    print(f"Error: Sweep directory '{self.sweep_dir}' already exists and is not empty.", file=sys.stderr)
                    print("Specify a different directory or pass --overwrite to replace existing data.", file=sys.stderr)
                    sys.exit(1)
                else:
                    print(f"[WARN] Sweep directory '{self.sweep_dir}' exists and is not empty. --overwrite specified: clearing previous bags and run directories...")
                    for item in os.listdir(self.sweep_dir):
                        item_path = os.path.join(self.sweep_dir, item)
                        if os.path.isdir(item_path) and item.startswith('run_'):
                            shutil.rmtree(item_path, ignore_errors=True)
                        elif item in ('sweep_metadata.csv', 'sim_launch.log'):
                            try:
                                os.remove(item_path)
                            except OSError:
                                pass

        os.makedirs(self.sweep_dir, exist_ok=True)
        self.bag_topics = resolve_bag_topics(self.args, self.ns)
        self.storage_id = getattr(self.args, 'storage_id', 'mcap') or 'mcap'
        
        self.metadata_file = os.path.join(self.sweep_dir, "sweep_metadata.csv")
        with open(self.metadata_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['run_id', 'start_x', 'start_y', 'goal_x', 'goal_y', 'status', 'elapsed_time'])

    def run(self):
        waypoints = []
        with open(self.args.waypoints, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                waypoints.append(row)
        
        if self.args.max_runs is not None:
            waypoints = waypoints[:self.args.max_runs]

        print(f"Starting sweep with {len(waypoints)} trajectories.")
        print(f"Output directory: {self.sweep_dir}")

        try:
            for i, wp in enumerate(waypoints):
                run_id = f"run_{i:03d}"
                print(f"\n{'='*50}\nStarting {run_id}\n{'='*50}")
                
                start_time = time.time()
                status = self.run_trajectory(run_id, wp)
                elapsed = time.time() - start_time
                
                with open(self.metadata_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([run_id, wp['start_x'], wp['start_y'], wp['goal_x'], wp['goal_y'], status, f"{elapsed:.2f}"])
                
                print(f"{run_id} finished with status: {status} in {elapsed:.2f}s")
                time.sleep(2.0)
        except KeyboardInterrupt:
            print("\nSweep interrupted by user. Cleaning up...")
            self.cleanup_orphans()
            sys.exit(130)

    def cleanup_orphans(self):
        # Clean up Gazebo and specific scripts
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
        subprocess.run(['pkill', '-9', '-f', 'log_state.py'], capture_output=True)
        
        # Clean up ROS 2 nodes, but never kill ourselves or any of our ancestors (e.g. wrapper shell)
        my_pid = os.getpid()
        protected_pids = {my_pid}
        
        try:
            curr = my_pid
            while curr > 1:
                out = subprocess.check_output(['ps', '-o', 'ppid=', '-p', str(curr)]).decode().strip()
                if not out:
                    break
                curr = int(out)
                protected_pids.add(curr)
        except Exception:
            pass

        try:
            out = subprocess.check_output(['pgrep', '-f', 'ros2']).decode().split()
            for p in out:
                pid = int(p)
                if pid not in protected_pids:
                    try:
                        os.kill(pid, 9)
                    except ProcessLookupError:
                        pass
        except (subprocess.CalledProcessError, ValueError):
            pass
        time.sleep(1.0)

    def run_trajectory(self, run_id, wp):
        raise NotImplementedError()


class ColdRestartRunner(SimulationRunner):
    def run_trajectory(self, run_id, wp):
        # 1. Pre-flight cleanup
        self.cleanup_orphans()
        
        run_dir = os.path.join(self.sweep_dir, run_id)
        os.makedirs(run_dir, exist_ok=True)
        bag_dir = os.path.join(run_dir, 'bag')
        
        # 2. Start Background Processes
        # log_state.py
        state_file = os.path.join(run_dir, 'state.jsonl')
        logger_proc = subprocess.Popen([
            'ros2', 'run', 'nav_worlds', 'log_state.py',
            '--odom_topic', f'/{self.ns}/platform/odom/filtered',
            '--cmd_vel_topic', f'/{self.ns}/cmd_vel',
            '--scan_topic', f'/{self.ns}/sensors/lidar2d_0/scan_filtered',
            '--target_frame', 'map',
            '--output', state_file
        ])

        try:
            with RosbagRecorder(
                bag_dir,
                self.bag_topics,
                storage_id=self.storage_id,
                use_sim_time=True
            ):
                # 3. Main Launch
                x = wp['start_x']
                y = wp['start_y']
                yaw = wp['start_yaw_rad']
                goal_x = wp['goal_x']
                goal_y = wp['goal_y']
                
                # If goal_yaw_rad isn't in the CSV, convert from goal_yaw_deg
                if 'goal_yaw_rad' in wp:
                    goal_yaw = wp['goal_yaw_rad']
                else:
                    goal_yaw = str(float(wp['goal_yaw_deg']) * 3.14159 / 180.0)

                headless_str = 'false' if self.args.gui else 'true'
                rviz_str = 'true' if self.args.rviz else 'false'
                
                launch_cmd = [
                    'ros2', 'launch', 'nav_worlds', 'a200_point_nav.launch.py',
                    f'world:={self.args.world}',
                    f'headless:={headless_str}',
                    f'rviz:={rviz_str}',
                    f'x:={x}', f'y:={y}', f'yaw:={yaw}',
                    f'goal_x:={goal_x}', f'goal_y:={goal_y}', f'goal_yaw:={goal_yaw}',
                    'run_goal:=true',
                    'slam:=false'
                ]
                
                print(f"Launching simulation: {' '.join(launch_cmd)}")
                try:
                    # send_goal.py has a timeout of 180s. We'll allow 400s max to account for bootup.
                    subprocess.run(launch_cmd, timeout=400)
                    status = 'completed'
                except subprocess.TimeoutExpired:
                    print("Launch command timed out!")
                    status = 'timeout'
                except Exception as e:
                    print(f"Launch command failed: {e}")
                    status = 'failed'
        finally:
            # 4. Teardown
            logger_proc.terminate()
            try:
                logger_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                logger_proc.kill()
            
        return status


class WarmRestartRunner(SimulationRunner):
    def __init__(self, args):
        super().__init__(args)
        self.launch_proc = None
        self.node = None
        self.navigator = None
        self.pub_cmd = None
        self.pub_init = None

    def init_ros(self, clock_timeout: float = 5.0, use_sim_time: bool = True):
        if rclpy is None:
            raise RuntimeError(
                "ROS 2 Python packages (rclpy, geometry_msgs, nav2_msgs, nav2_simple_commander) "
                "not found. Did you source setup.bash?"
            )
        if not rclpy.ok():
            rclpy.init()

        from rclpy.parameter import Parameter
        self.navigator = BasicNavigator(node_name='sweep_navigator', namespace=self.ns)
        self.navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
        self.node = self.navigator

        self.pub_cmd = self.node.create_publisher(TwistStamped, f'/{self.ns}/cmd_vel', 10)
        self.pub_init = self.node.create_publisher(PoseWithCovarianceStamped, f'/{self.ns}/initialpose', 10)
        self.amcl_set_initial_pose_srv = self.node.create_client(SetInitialPose, f'/{self.ns}/set_initial_pose')
        self.amcl_nomotion_update_srv = self.node.create_client(Empty, f'/{self.ns}/request_nomotion_update')
        try:
            from ros_gz_interfaces.srv import SetEntityPose
            self.set_pose_client = self.node.create_client(SetEntityPose, f'/world/{self.args.world}/set_pose')
        except ImportError:
            self.set_pose_client = None

        if use_sim_time and clock_timeout > 0.0:
            start_clock_wait = time.time()
            while self.node.get_clock().now().nanoseconds == 0:
                if time.time() - start_clock_wait > clock_timeout:
                    print(f"[DEBUG] Simulation /clock did not become active within {clock_timeout:.1f}s. Proceeding...")
                    break
                if self.launch_proc and self.launch_proc.poll() is not None:
                    raise RuntimeError("Simulation launch exited prematurely!")
                rclpy.spin_once(self.node, timeout_sec=0.1)

    def teleport_gazebo(self, x, y, z, yaw_rad):
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        model_name = getattr(self.args, 'model_name', None) or f"{self.ns}/robot"

        # 1. Try native ROS 2 service bridge
        if hasattr(self, 'set_pose_client') and self.set_pose_client is not None:
            try:
                from ros_gz_interfaces.srv import SetEntityPose
                from ros_gz_interfaces.msg import Entity
                if self.set_pose_client.wait_for_service(timeout_sec=0.5):
                    req = SetEntityPose.Request()
                    req.entity.name = model_name
                    req.entity.type = Entity.MODEL
                    req.pose.position.x = float(x)
                    req.pose.position.y = float(y)
                    req.pose.position.z = float(z)
                    req.pose.orientation.z = qz
                    req.pose.orientation.w = qw
                    future = self.set_pose_client.call_async(req)
                    rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                    if future.done() and future.result() is not None:
                        if future.result().success:
                            return True
                        else:
                            print(f"[DEBUG] SetEntityPose returned success=False for '{model_name}'. Falling back to gz CLI...")
            except Exception as e:
                print(f"[DEBUG] ROS 2 SetEntityPose service call failed: {e}. Falling back to gz CLI...")

        # 2. Fallback to gz CLI
        gz_bin = shutil.which('gz') or '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'
        candidate_names = [model_name, f"{self.ns}/robot", self.ns, "robot"]
        for name in candidate_names:
            req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}'
            res = subprocess.run([
                gz_bin, 'service', '-s', f'/world/{self.args.world}/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', req
            ], capture_output=True, text=True)
            if 'data: true' in res.stdout:
                return True
        return False

    def reset_environment(self, x, y, yaw_rad):
        print(f"Resetting environment to ({x}, {y}, yaw={yaw_rad:.3f} rad)...")
        # 1. Zero cmd_vel
        twist = TwistStamped()
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        for _ in range(3):
            self.pub_cmd.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # 2. Teleport Gazebo model
        ok = self.teleport_gazebo(x, y, 0.3, yaw_rad)
        if not ok:
            print("[WARN] Gazebo set_pose service call did not confirm success. Check world or model name.")

        # 3. Publish AMCL initialpose and reset robot_localization EKF
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        # Try AMCL set_initial_pose service first
        if hasattr(self, 'amcl_set_initial_pose_srv') and self.amcl_set_initial_pose_srv.wait_for_service(timeout_sec=0.2):
            try:
                req_init = SetInitialPose.Request()
                req_init.pose = msg
                future_init = self.amcl_set_initial_pose_srv.call_async(req_init)
                rclpy.spin_until_future_complete(self.node, future_init, timeout_sec=1.0)
            except Exception as e:
                print(f"[DEBUG] set_initial_pose service call skipped: {e}")

        # Always publish initialpose to ensure all listeners (RViz, EKF) receive it
        for _ in range(3):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pub_init.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Trigger AMCL nomotion update to force map -> odom transform broadcast without waiting for wheel motion
        if hasattr(self, 'amcl_nomotion_update_srv') and self.amcl_nomotion_update_srv.wait_for_service(timeout_sec=0.2):
            try:
                future_nomotion = self.amcl_nomotion_update_srv.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self.node, future_nomotion, timeout_sec=1.0)
            except Exception as e:
                print(f"[DEBUG] request_nomotion_update service call skipped: {e}")

        try:
            from robot_localization.srv import SetPose
            ekf_client = self.node.create_client(SetPose, f'/{self.ns}/set_pose')
            if ekf_client.wait_for_service(timeout_sec=1.0):
                req = SetPose.Request()
                req.pose = msg
                future = ekf_client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        except Exception as e:
            print(f"[DEBUG] EKF set_pose skipped or failed: {e}")

        # 4. Clear costmaps and verify completion
        for client, name in [
            (self.navigator.clear_costmap_local_srv, 'local costmap'),
            (self.navigator.clear_costmap_global_srv, 'global costmap')
        ]:
            if client.wait_for_service(timeout_sec=2.0):
                req = ClearEntireCostmap.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                if future.done() and future.result() is not None:
                    print(f"Successfully cleared {name}.")
                else:
                    print(f"[WARN] Failed to clear {name}.")
            else:
                print(f"[WARN] {name} clear service unavailable.")

    def wait_for_nav2_ready(self, timeout_sec=180.0):
        print(f"Waiting for Nav2 to become active in namespace /{self.ns}...")
        start_wait = time.time()
        nodes_to_check = ['bt_navigator', 'planner_server', 'controller_server']

        for node_name in nodes_to_check:
            node_service = f'{node_name}/get_state'
            state_client = self.navigator.create_client(GetState, node_service)

            while time.time() - start_wait < timeout_sec:
                if self.launch_proc and self.launch_proc.poll() is not None:
                    raise RuntimeError(
                        f"Simulation launch exited prematurely (code {self.launch_proc.returncode})!"
                    )
                if state_client.wait_for_service(timeout_sec=1.0):
                    break
            else:
                self.navigator.destroy_client(state_client)
                raise TimeoutError(f"Service {node_service} unavailable within {timeout_sec}s")

            req = GetState.Request()
            state = 'unknown'
            while state != 'active':
                if time.time() - start_wait > timeout_sec:
                    self.navigator.destroy_client(state_client)
                    raise TimeoutError(
                        f"Node {node_name} failed to reach active state within {timeout_sec}s"
                    )
                if self.launch_proc and self.launch_proc.poll() is not None:
                    self.navigator.destroy_client(state_client)
                    raise RuntimeError(
                        f"Simulation launch exited prematurely (code {self.launch_proc.returncode})!"
                    )
                future = state_client.call_async(req)
                rclpy.spin_until_future_complete(self.navigator, future, timeout_sec=1.0)
                if future.done() and future.result() is not None:
                    state = future.result().current_state.label
                time.sleep(0.5)

            self.navigator.destroy_client(state_client)
            print(f"  Nav2 component '{node_name}' is active.")

        print("Simulation stack and Nav2 are READY for use!")
        return True

    def run(self):
        waypoints = []
        with open(self.args.waypoints, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                waypoints.append(row)
        
        if self.args.max_runs is not None:
            waypoints = waypoints[:self.args.max_runs]

        print(f"Starting warm reset sweep with {len(waypoints)} trajectories.")
        print(f"Output directory: {self.sweep_dir}")

        # 1. Clean previous runs
        self.cleanup_orphans()

        # 2. Launch persistent simulation + nav2 stack
        first_wp = waypoints[0] if waypoints else {'start_x': '0.0', 'start_y': '0.0', 'start_yaw_rad': '0.0'}
        headless_str = 'false' if self.args.gui else 'true'
        rviz_str = 'true' if self.args.rviz else 'false'
        
        launch_log_path = os.path.join(self.sweep_dir, 'sim_launch.log')
        launch_cmd = [
            'ros2', 'launch', 'nav_worlds', 'a200_point_nav.launch.py',
            f'world:={self.args.world}',
            f'headless:={headless_str}',
            f'rviz:={rviz_str}',
            f'x:={first_wp["start_x"]}',
            f'y:={first_wp["start_y"]}',
            f'yaw:={first_wp.get("start_yaw_rad", "0.0")}',
            'run_goal:=false',
            'slam:=false'
        ]

        print(f"Launching simulation stack (logging to {launch_log_path})...")
        launch_log_file = open(launch_log_path, 'w')
        self.launch_proc = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )

        def stream_output():
            for line in self.launch_proc.stdout:
                launch_log_file.write(line)
                launch_log_file.flush()
                if getattr(self.args, 'verbose', False):
                    sys.stdout.write(line)
                    sys.stdout.flush()

        self.stream_thread = threading.Thread(target=stream_output, daemon=True)
        self.stream_thread.start()

        try:
            self.init_ros()
            self.wait_for_nav2_ready(timeout_sec=180.0)

            # 3. Trajectory Loop
            for i, wp in enumerate(waypoints):
                run_id = f"run_{i:03d}"
                print(f"\n{'='*50}\nStarting {run_id}\n{'='*50}")

                start_time = time.time()
                status = self.run_trajectory(run_id, wp)
                elapsed = time.time() - start_time

                with open(self.metadata_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([run_id, wp['start_x'], wp['start_y'], wp['goal_x'], wp['goal_y'], status, f"{elapsed:.2f}"])

                print(f"{run_id} finished with status: {status} in {elapsed:.2f}s")
                time.sleep(1.0)

        except KeyboardInterrupt:
            print("\nSweep interrupted by user. Cleaning up...")
        finally:
            if self.node:
                self.node.destroy_node()
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
            if self.launch_proc:
                print("Stopping simulation stack...")
                self.launch_proc.terminate()
                try:
                    self.launch_proc.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    self.launch_proc.kill()
            launch_log_file.close()
            self.cleanup_orphans()

    def run_trajectory(self, run_id, wp):
        # 1. Reset Environment
        yaw = float(wp.get('start_yaw_rad', 0.0))
        self.reset_environment(float(wp['start_x']), float(wp['start_y']), yaw)

        run_dir = os.path.join(self.sweep_dir, run_id)
        os.makedirs(run_dir, exist_ok=True)
        bag_dir = os.path.join(run_dir, 'bag')

        # 2. Start Background Loggers
        state_file = os.path.join(run_dir, 'state.jsonl')
        logger_proc = subprocess.Popen([
            'ros2', 'run', 'nav_worlds', 'log_state.py',
            '--odom_topic', f'/{self.ns}/platform/odom/filtered',
            '--cmd_vel_topic', f'/{self.ns}/cmd_vel',
            '--scan_topic', f'/{self.ns}/sensors/lidar2d_0/scan_filtered',
            '--target_frame', 'map',
            '--output', state_file
        ])

        try:
            with RosbagRecorder(
                bag_dir,
                self.bag_topics,
                storage_id=self.storage_id,
                use_sim_time=True
            ):
                # 3. Execute Goal in-process via BasicNavigator
                status = self.execute_nav_goal(wp)
        finally:
            # 4. Teardown Loggers
            logger_proc.terminate()
            try:
                logger_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                logger_proc.kill()

            # Clean up any orphaned log_state
            subprocess.run(['pkill', '-9', '-f', 'log_state.py'], capture_output=True)

        return status

    def execute_nav_goal(self, wp, timeout_sim: float = 180.0) -> str:
        goal_x = float(wp['goal_x'])
        goal_y = float(wp['goal_y'])
        if 'goal_yaw_rad' in wp:
            goal_yaw_rad = float(wp['goal_yaw_rad'])
        elif 'goal_yaw_deg' in wp:
            goal_yaw_rad = math.radians(float(wp['goal_yaw_deg']))
        else:
            goal_yaw_rad = 0.0

        publish_goal_marker(
            self.navigator, goal_x, goal_y,
            frame='map', namespace=self.ns, world=self.args.world
        )

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.z = math.sin(goal_yaw_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(goal_yaw_rad / 2.0)

        # Pre-flight check: ensure path is plannable from current robot pose before issuing navigation goal
        if (
            hasattr(self.navigator, 'compute_path_to_pose_client')
            and self.navigator.compute_path_to_pose_client.server_is_ready()
        ):
            print("Verifying path plan from current pose to goal...")
            start_check = time.time()
            path_confirmed = False
            while time.time() - start_check < 5.0:
                try:
                    path = self.navigator.getPath(
                        start=PoseStamped(), goal=goal_pose, use_start=False
                    )
                    if path is not None and len(path.poses) > 0:
                        path_confirmed = True
                        break
                except Exception as e:
                    print(f"[DEBUG] Pre-flight getPath query failed: {e}")
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if not path_confirmed:
                print("[DEBUG] Pre-flight path check timed out; proceeding with goToPose...")
            else:
                print(f"Path verified ({len(path.poses)} waypoints).")

        print(
            f'Executing goal: ({goal_x:.2f}, {goal_y:.2f}, yaw={goal_yaw_rad:.3f} rad)...'
        )
        accepted = self.navigator.goToPose(goal_pose)
        if not accepted:
            print('Goal rejected by Nav2!')
            return 'failed'

        clock = self.navigator.get_clock()
        start_sim = clock.now()
        start_wall = time.time()
        timeout_wall = max(timeout_sim * 2.5, timeout_sim + 60.0)
        last_feedback_time = 0.0
        status = 'failed'

        while not self.navigator.isTaskComplete():
            now_wall = time.time()
            feedback = self.navigator.getFeedback()
            if feedback and (now_wall - last_feedback_time >= 5.0):
                last_feedback_time = now_wall
                print(f'  distance remaining: {feedback.distance_remaining:.2f} m')

            elapsed_sim = (clock.now() - start_sim).nanoseconds * 1e-9
            elapsed_wall = time.time() - start_wall
            if (elapsed_sim > timeout_sim and elapsed_sim > 0.0) or (elapsed_wall > timeout_wall):
                print(
                    f'Goal timed out after {elapsed_sim:.1f}s sim ({elapsed_wall:.1f}s wall)!'
                )
                self.navigator.cancelTask()
                return 'timeout'
            time.sleep(0.1)

        result = self.navigator.getResult()
        elapsed_sim = (clock.now() - start_sim).nanoseconds * 1e-9
        elapsed_wall = time.time() - start_wall
        if result == TaskResult.SUCCEEDED:
            print(
                f'Goal SUCCEEDED after {elapsed_sim:.1f}s sim ({elapsed_wall:.1f}s wall)'
            )
            status = 'completed'
        elif result == TaskResult.CANCELED:
            print(
                f'Goal CANCELED after {elapsed_sim:.1f}s sim ({elapsed_wall:.1f}s wall)'
            )
            status = 'canceled'
        else:
            print(
                f'Goal FAILED after {elapsed_sim:.1f}s sim ({elapsed_wall:.1f}s wall)'
            )
            status = 'failed'

        return status


def main():
    parser = argparse.ArgumentParser(description="Automated Data Collection Sweep")
    parser.add_argument('--waypoints', type=str, default='data/warehouse_waypoints.csv', help='Path to waypoints CSV')
    parser.add_argument('--world', type=str, default='warehouse', help='Gazebo world name')
    parser.add_argument('--output_dir', type=str, default='data/dataset_output', help='Base output directory')
    parser.add_argument('--max_runs', type=int, default=None, help='Max trajectories to run')
    parser.add_argument('--gui', action='store_true', help='Run Gazebo with GUI (headless:=false)')
    parser.add_argument('--rviz', action='store_true', help='Run RViz visualization (rviz:=true)')
    parser.add_argument('--include-camera', action='store_true', help='Include camera topics in rosbag')
    parser.add_argument('--disable-lidar2d', action='store_true', help='Disable 2D Lidar in rosbag')
    parser.add_argument('--sweep-dir', type=str, default=None, help='Explicit directory for this sweep (overrides output_dir auto-timestamping)')
    parser.add_argument('--bag-profile', type=str, default='standard', choices=['minimal', 'standard', 'perception', 'full'], help='Rosbag topic profile preset (default: standard)')
    parser.add_argument('--storage-id', type=str, default='mcap', choices=['mcap', 'sqlite3'], help='Rosbag storage backend (default: mcap)')
    parser.add_argument('--add-topics', nargs='*', default=None, help='Additional topics to record in rosbag')
    parser.add_argument('--custom-topics', nargs='*', default=None, help='Explicit full list of topics to record in rosbag (overrides profile)')
    parser.add_argument('--cold-restart', action='store_true', help='Use cold restart (relaunch simulation for each trajectory instead of warm reset)')
    parser.add_argument('--warm-reset', action='store_true', default=True, help='Use warm reset (default: keeps simulation running between trajectories)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Stream simulation launch output directly to the terminal in real time')
    parser.add_argument('--model-name', type=str, default='a200_0000/robot', help='Gazebo model name for robot teleportation')
    parser.add_argument('--overwrite', action='store_true', help='Allow overwriting existing sweep directory, clearing old bags and run data')
    
    args = parser.parse_args()
    
    if args.cold_restart:
        runner = ColdRestartRunner(args)
    else:
        runner = WarmRestartRunner(args)
    runner.run()


if __name__ == '__main__':
    main()
