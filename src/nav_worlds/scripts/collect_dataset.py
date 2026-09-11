#!/usr/bin/env python3
"""End-to-End Autonomous Data Collection Pipeline.

Ties together:
1. Dynamic, collision-free waypoint trajectory generation.
2. Interactive visual preview (xdg-open) and user confirmation gate.
3. Automated simulation sweep execution (warm reset default).
4. Configurable rosbag topic profiles and synchronized JSONL state logging.
"""

import os
import sys
import csv
import shutil
import argparse
import subprocess
from datetime import datetime

# Allow importing peer scripts
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_waypoints import generate_waypoints, resolve_default_map


def resolve_world_map(world_name: str, explicit_map: str = '') -> str:
    """Find the map YAML file corresponding to a world name."""
    if explicit_map and os.path.exists(explicit_map):
        return explicit_map

    # 1. Look in nav_worlds package
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_nav = get_package_share_directory('nav_worlds')
        cand = os.path.join(pkg_nav, 'maps', f'{world_name}.yaml')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass

    # 2. Look in local workspace source
    ws_cand = os.path.expanduser(f'~/ros2_ws/src/nav_worlds/maps/{world_name}.yaml')
    if os.path.exists(ws_cand):
        return ws_cand

    # 3. Fallback to clearpath_nav2_demos
    default_demo = resolve_default_map()
    if default_demo and world_name in default_demo:
        return default_demo

    return ''


def prompt_user_confirmation(preview_path: str, num_trajectories: int) -> bool:
    """Launch xdg-open if GUI display exists and prompt for CLI approval."""
    has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    xdg_bin = shutil.which('xdg-open')

    print("\n" + "=" * 60)
    print(f"Waypoints generated: {num_trajectories}")
    print(f"Preview image: {preview_path}")

    viewer_proc = None
    if has_display and xdg_bin:
        try:
            print("Opening waypoint trajectory preview with xdg-open...")
            viewer_proc = subprocess.Popen(
                [xdg_bin, preview_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            print(f"Notice: Could not launch xdg-open: {e}")
    else:
        print("Note: Running without graphical display. Open the preview image manually to inspect.")

    print("=" * 60)

    try:
        ans = input("Proceed with simulation sweep using these trajectories? [Y/n]: ").strip().lower()
    except (KeyboardInterrupt, EOFError):
        print("\nAborted by user.")
        return False

    return ans in ('', 'y', 'yes')


def main():
    parser = argparse.ArgumentParser(description="End-to-End Dataset Collection Pipeline")

    # Trajectory Generation Arguments
    gen_group = parser.add_argument_group("Trajectory Generation")
    gen_group.add_argument('-n', '--num-trajectories', type=int, default=5,
                           help='Number of waypoint trajectories to generate (default: 5)')
    gen_group.add_argument('--world', type=str, default='warehouse',
                           help='Gazebo world name (default: warehouse)')
    gen_group.add_argument('--map-yaml', type=str, default='',
                           help='Explicit path to map YAML file (default: auto-resolved from world)')
    gen_group.add_argument('--min-dist', type=float, default=4.0,
                           help='Minimum trajectory Euclidean distance in meters (default: 4.0)')
    gen_group.add_argument('--max-dist', type=float, default=25.0,
                           help='Maximum trajectory Euclidean distance in meters (default: 25.0)')
    gen_group.add_argument('--clearance', type=float, default=0.6,
                           help='Obstacle clearance radius in meters (default: 0.6)')
    gen_group.add_argument('--similarity-thresh', type=float, default=2.0,
                           help='Minimum separation between distinct sampled trajectory pairs (default: 2.0)')
    gen_group.add_argument('--seed', type=int, default=None,
                           help='Optional random seed for reproducible waypoint generation')
    gen_group.add_argument('--random-yaw', action='store_true',
                           help='Randomize start/goal yaw orientations instead of facing the goal')
    gen_group.add_argument('--waypoints', type=str, default='',
                           help='Skip generation and use an existing waypoints CSV file')

    # Output Directory Arguments
    out_group = parser.add_argument_group("Output Configuration")
    out_group.add_argument('-o', '--output-dir', type=str, default='',
                           help='Destination folder for the dataset (default: data/dataset_output/sweep_YYYYMMDD_HHMMSS)')

    # Interactive Confirmation Arguments
    ui_group = parser.add_argument_group("Interactive Confirmation")
    ui_group.add_argument('-y', '--no-prompt', action='store_true',
                          help='Automatically approve generated waypoints without opening xdg-open or prompting')

    # Sweep & Bag Configuration Arguments
    sweep_group = parser.add_argument_group("Sweep & Rosbag Configuration")
    sweep_group.add_argument('--bag-profile', type=str, default='standard',
                             choices=['minimal', 'standard', 'perception', 'full'],
                             help='Rosbag recording profile preset (default: standard)')
    sweep_group.add_argument('--add-topics', nargs='*', default=None,
                             help='Additional topics to record in rosbag')
    sweep_group.add_argument('--custom-topics', nargs='*', default=None,
                             help='Explicit list of topics to record in rosbag (overrides profile)')
    sweep_group.add_argument('--cold-restart', action='store_true',
                             help='Use cold restart instead of the default warm reset')
    sweep_group.add_argument('--gui', action='store_true',
                             help='Launch Gazebo with GUI enabled (headless:=false)')
    sweep_group.add_argument('--rviz', action='store_true',
                             help='Launch RViz visualization (rviz:=true)')
    sweep_group.add_argument('-v', '--verbose', action='store_true',
                             help='Stream underlying simulation and node outputs to terminal in real time')
    sweep_group.add_argument('--overwrite', action='store_true',
                             help='Allow overwriting existing destination directory, deleting previous run bags and logs')

    args = parser.parse_args()

    # 1. Setup Output Directory
    if args.output_dir:
        dest_dir = os.path.abspath(args.output_dir)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        dest_dir = os.path.abspath(os.path.join('data', 'dataset_output', f"sweep_{timestamp}"))

    if os.path.exists(dest_dir) and os.listdir(dest_dir):
        if not args.overwrite:
            print(f"Error: Destination directory '{dest_dir}' already exists and is not empty.", file=sys.stderr)
            print("Specify a different --output-dir or pass --overwrite to replace existing data.", file=sys.stderr)
            sys.exit(1)
        else:
            print(f"[WARN] Destination directory '{dest_dir}' exists and is not empty. --overwrite specified: cleaning previous run data...")
            for item in os.listdir(dest_dir):
                item_path = os.path.join(dest_dir, item)
                if os.path.isdir(item_path) and item.startswith('run_'):
                    shutil.rmtree(item_path, ignore_errors=True)
                elif item in ('sweep_metadata.csv', 'sim_launch.log', 'waypoints.csv', 'waypoints_preview.png'):
                    try:
                        os.remove(item_path)
                    except OSError:
                        pass

    os.makedirs(dest_dir, exist_ok=True)
    print(f"Dataset destination directory: {dest_dir}")

    # 2. Waypoint Generation or Ingestion
    waypoints_csv = os.path.join(dest_dir, 'waypoints.csv')
    preview_png = os.path.join(dest_dir, 'waypoints_preview.png')

    if args.waypoints:
        if not os.path.exists(args.waypoints):
            print(f"Error: Specified waypoints file does not exist: {args.waypoints}", file=sys.stderr)
            sys.exit(1)
        shutil.copyfile(args.waypoints, waypoints_csv)
        print(f"Copied existing waypoints from {args.waypoints} to {waypoints_csv}")
    else:
        map_yaml = resolve_world_map(args.world, args.map_yaml)
        if not map_yaml:
            print(f"Error: Could not resolve map YAML for world '{args.world}'. Please provide --map-yaml.", file=sys.stderr)
            sys.exit(1)

        print(f"Generating {args.num_trajectories} waypoints from {map_yaml}...")
        waypoints = generate_waypoints(
            map_yaml=map_yaml,
            num_samples=args.num_trajectories,
            min_distance=args.min_dist,
            max_distance=args.max_dist,
            clearance_m=args.clearance,
            similarity_thresh=args.similarity_thresh,
            face_goal=not args.random_yaw,
            seed=args.seed,
            preview_path=preview_png
        )

        if not waypoints:
            print("Error: Waypoint generation failed to produce valid trajectories.", file=sys.stderr)
            sys.exit(1)

        # Save to waypoints.csv inside dest_dir
        fields = ['id', 'start_x', 'start_y', 'start_yaw_rad', 'start_yaw_deg', 'goal_x', 'goal_y', 'goal_yaw_deg', 'distance']
        with open(waypoints_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for wp in waypoints:
                row = {k: wp[k] for k in fields}
                writer.writerow(row)

        print(f"Successfully wrote {len(waypoints)} waypoints to {waypoints_csv}")

        # 3. Interactive Visual Approval
        if not args.no_prompt and os.path.exists(preview_png):
            proceed = prompt_user_confirmation(preview_png, len(waypoints))
            if not proceed:
                print("Aborting sweep execution. Generated waypoints preserved.")
                sys.exit(0)

    # 4. Invoke Sweep Orchestrator (run_sweep.py)
    run_sweep_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'run_sweep.py')
    sweep_cmd = [
        sys.executable, run_sweep_script,
        '--waypoints', waypoints_csv,
        '--sweep-dir', dest_dir,
        '--world', args.world,
        '--bag-profile', args.bag_profile
    ]

    if args.cold_restart:
        sweep_cmd.append('--cold-restart')
    if args.gui:
        sweep_cmd.append('--gui')
    if args.rviz:
        sweep_cmd.append('--rviz')
    if args.add_topics:
        sweep_cmd.extend(['--add-topics'] + args.add_topics)
    if args.custom_topics:
        sweep_cmd.extend(['--custom-topics'] + args.custom_topics)
    if args.verbose:
        sweep_cmd.append('--verbose')
    if args.overwrite:
        sweep_cmd.append('--overwrite')

    print("\n" + "=" * 60)
    print("Launching simulation sweep...")
    print(f"Command: {' '.join(sweep_cmd)}")
    print("=" * 60 + "\n")

    res = subprocess.run(sweep_cmd)

    # 5. Print Final Summary
    meta_csv = os.path.join(dest_dir, 'sweep_metadata.csv')
    if os.path.exists(meta_csv):
        print("\n" + "=" * 60)
        print(f"Dataset Collection Completed!")
        print(f"Outputs located at: {dest_dir}")
        print("Summary of runs:")
        with open(meta_csv, 'r') as f:
            for line in f:
                print(f"  {line.strip()}")
        print("=" * 60)

    sys.exit(res.returncode)


if __name__ == '__main__':
    main()
