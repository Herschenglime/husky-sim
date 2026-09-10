#!/usr/bin/env python3
import os
import sys
import csv
import time
import argparse
import subprocess
from datetime import datetime

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
    
    args = parser.parse_args()
    
    runner = ColdRestartRunner(args)
    runner.run()

if __name__ == '__main__':
    main()
