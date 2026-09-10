#!/usr/bin/env python3
import os
import sys
import csv
import time
import math
import shutil
import argparse
import subprocess
from datetime import datetime

try:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
    from nav2_msgs.srv import ClearEntireCostmap
except ImportError:
    rclpy = None

class SimulationRunner:
    def __init__(self, args):
        self.args = args
        self.ns = "a200_0000"
        
        # Setup output directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.sweep_dir = os.path.join(self.args.output_dir, f"sweep_{timestamp}")
        os.makedirs(self.sweep_dir, exist_ok=True)
        
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
        pass

    def run_trajectory(self, run_id, wp):
        raise NotImplementedError()


class ColdRestartRunner(SimulationRunner):
    def cleanup_orphans(self):
        # Clean up Gazebo and specific scripts
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
        subprocess.run(['pkill', '-9', '-f', 'log_state.py'], capture_output=True)
        
        # Clean up ROS 2 nodes, but never kill ourselves or our parent process
        my_pid = os.getpid()
        parent_pid = os.getppid()
        try:
            out = subprocess.check_output(['pgrep', '-f', 'ros2']).decode().split()
            for p in out:
                pid = int(p)
                if pid not in (my_pid, parent_pid):
                    try:
                        os.kill(pid, 9)
                    except ProcessLookupError:
                        pass
        except (subprocess.CalledProcessError, ValueError):
            pass
        time.sleep(1.0)

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
            '--output', state_file
        ])

        # rosbag
        bag_topics = [
            f'/{self.ns}/platform/odom/filtered',
            f'/{self.ns}/cmd_vel',
            f'/{self.ns}/tf',
            f'/{self.ns}/tf_static',
            f'/{self.ns}/joint_states',
            f'/{self.ns}/plan',
            f'/{self.ns}/goal_pose',
            '/clock'
        ]
        if not self.args.disable_lidar2d:
            bag_topics.append(f'/{self.ns}/sensors/lidar2d_0/scan_filtered')
        if self.args.include_camera:
            bag_topics.extend([
                f'/{self.ns}/sensors/camera_0/color/image',
                f'/{self.ns}/sensors/camera_0/color/camera_info'
            ])
            
        bag_cmd = ['ros2', 'bag', 'record', '-o', bag_dir, '--use-sim-time'] + bag_topics
        bag_proc = subprocess.Popen(bag_cmd)
        
        # Give them a second to initialize
        time.sleep(2.0)
        
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
            
        # 4. Teardown
        logger_proc.terminate()
        bag_proc.terminate()
        
        try:
            logger_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            logger_proc.kill()
            
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()
            
        return status


class WarmRestartRunner(SimulationRunner):
    def __init__(self, args):
        super().__init__(args)
        self.launch_proc = None
        self.node = None
        self.pub_cmd = None
        self.pub_init = None

    def cleanup_orphans(self):
        # Clean up Gazebo and specific scripts
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
        subprocess.run(['pkill', '-9', '-f', 'log_state.py'], capture_output=True)
        
        # Clean up ROS 2 nodes, but never kill ourselves or our parent process
        my_pid = os.getpid()
        parent_pid = os.getppid()
        try:
            out = subprocess.check_output(['pgrep', '-f', 'ros2']).decode().split()
            for p in out:
                pid = int(p)
                if pid not in (my_pid, parent_pid):
                    try:
                        os.kill(pid, 9)
                    except ProcessLookupError:
                        pass
        except (subprocess.CalledProcessError, ValueError):
            pass
        time.sleep(1.0)

    def init_ros(self):
        if rclpy is None:
            raise RuntimeError("ROS 2 Python packages (rclpy, geometry_msgs, nav2_msgs) not found. Did you source setup.bash?")
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('warm_reset_helper')
        self.pub_cmd = self.node.create_publisher(TwistStamped, f'/{self.ns}/cmd_vel', 10)
        self.pub_init = self.node.create_publisher(PoseWithCovarianceStamped, f'/{self.ns}/initialpose', 10)

    def teleport_gazebo(self, x, y, z, yaw_rad):
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        gz_bin = shutil.which('gz') or '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'
        candidate_names = [f"{self.ns}/robot", self.ns, "robot"]
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

        # 3. Publish AMCL initialpose
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

        for _ in range(3):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pub_init.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # 4. Clear costmaps
        for srv_name in [f'/{self.ns}/local_costmap/clear_entirely_local_costmap',
                         f'/{self.ns}/global_costmap/clear_entirely_global_costmap']:
            client = self.node.create_client(ClearEntireCostmap, srv_name)
            if client.wait_for_service(timeout_sec=2.0):
                req = ClearEntireCostmap.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        # Settle
        time.sleep(1.5)

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
        self.launch_proc = subprocess.Popen(launch_cmd, stdout=launch_log_file, stderr=subprocess.STDOUT)

        try:
            # Wait for Nav2 bringup to be READY
            print("Waiting for simulation & Nav2 to become READY...")
            start_wait = time.time()
            ready = False
            while time.time() - start_wait < 180.0:
                if self.launch_proc.poll() is not None:
                    raise RuntimeError("Simulation launch exited prematurely!")
                if os.path.exists(launch_log_path):
                    with open(launch_log_path, 'r', errors='ignore') as f:
                        if '[a200_point_nav] READY' in f.read():
                            ready = True
                            break
                time.sleep(1.0)

            if not ready:
                raise TimeoutError("Timed out waiting for simulation stack to be READY.")

            print("Simulation stack is READY!")
            time.sleep(2.0)
            self.init_ros()

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
            '--output', state_file
        ])

        bag_topics = [
            f'/{self.ns}/platform/odom/filtered',
            f'/{self.ns}/cmd_vel',
            f'/{self.ns}/tf',
            f'/{self.ns}/tf_static',
            f'/{self.ns}/joint_states',
            f'/{self.ns}/plan',
            f'/{self.ns}/goal_pose',
            '/clock'
        ]
        if not self.args.disable_lidar2d:
            bag_topics.append(f'/{self.ns}/sensors/lidar2d_0/scan_filtered')
        if self.args.include_camera:
            bag_topics.extend([
                f'/{self.ns}/sensors/camera_0/color/image',
                f'/{self.ns}/sensors/camera_0/color/camera_info'
            ])
            
        bag_cmd = ['ros2', 'bag', 'record', '-o', bag_dir, '--use-sim-time'] + bag_topics
        bag_proc = subprocess.Popen(bag_cmd)
        time.sleep(1.0)

        # 3. Execute Goal via send_goal.py
        if 'goal_yaw_deg' in wp:
            goal_yaw = wp['goal_yaw_deg']
        elif 'goal_yaw_rad' in wp:
            goal_yaw = str(math.degrees(float(wp['goal_yaw_rad'])))
        else:
            goal_yaw = '0.0'

        goal_cmd = [
            'ros2', 'run', 'nav_worlds', 'send_goal.py',
            str(wp['goal_x']), str(wp['goal_y']), str(goal_yaw),
            '--ns', self.ns,
            '--world', self.args.world,
            '--use-sim-time',
            '--timeout', '180'
        ]
        print(f"Executing goal: {' '.join(goal_cmd)}")
        try:
            res = subprocess.run(goal_cmd, timeout=240)
            status = 'completed' if res.returncode == 0 else 'failed'
        except subprocess.TimeoutExpired:
            print("Goal execution timed out!")
            status = 'timeout'
        except Exception as e:
            print(f"Goal execution failed: {e}")
            status = 'failed'

        # 4. Teardown Loggers
        logger_proc.terminate()
        bag_proc.terminate()

        try:
            logger_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            logger_proc.kill()
        try:
            bag_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bag_proc.kill()

        # Clean up any orphaned log_state
        subprocess.run(['pkill', '-9', '-f', 'log_state.py'], capture_output=True)

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
    parser.add_argument('--cold-restart', action='store_true', help='Use cold restart (relaunch simulation for each trajectory instead of warm reset)')
    parser.add_argument('--warm-reset', action='store_true', default=True, help='Use warm reset (default: keeps simulation running between trajectories)')
    
    args = parser.parse_args()
    
    if args.cold_restart:
        runner = ColdRestartRunner(args)
    else:
        runner = WarmRestartRunner(args)
    runner.run()


if __name__ == '__main__':
    main()
