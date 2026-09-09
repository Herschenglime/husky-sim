#!/usr/bin/env python3
"""Generate reachable, clearance-verified start and goal waypoints from a ROS 2 static map.

Parses a static map YAML/PGM, applies safety clearance erosion, extracts the
largest navigable connected component, and samples dissimilar start/goal waypoint
pairs. See doc/SIMULATION_SWEEP.md for design rationale and architecture.
"""

import argparse
import csv
import math
import os
import random
import sys

import cv2
import numpy as np
import yaml


def resolve_default_map() -> str:
    """Find default warehouse map from clearpath_nav2_demos if available."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_demos = get_package_share_directory('clearpath_nav2_demos')
        cand = os.path.join(pkg_demos, 'maps', 'warehouse.yaml')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    # Fallback to local source path
    local_cand = os.path.expanduser('~/ros2_ws/src/clearpath_nav2_demos/maps/warehouse.yaml')
    if os.path.exists(local_cand):
        return local_cand
    return ''


def load_map(yaml_path: str):
    """Load map metadata and image array."""
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Map YAML not found: {yaml_path}")

    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    map_dir = os.path.dirname(os.path.abspath(yaml_path))
    image_rel = meta.get('image', '')
    image_path = image_rel if os.path.isabs(image_rel) else os.path.join(map_dir, image_rel)

    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Map image not found: {image_path}")

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"Could not read map image from {image_path}")

    resolution = float(meta['resolution'])
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    origin_x, origin_y = float(origin[0]), float(origin[1])
    origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0
    negate = int(meta.get('negate', 0))
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))

    return img, meta, resolution, (origin_x, origin_y, origin_yaw), negate, occupied_thresh, free_thresh


def grid_to_world(col: int, row: int, height: int, resolution: float, origin: tuple) -> tuple:
    """Convert grid image coordinates (col, row) to world coordinates (x, y).

    In ROS map convention, origin is at bottom-left corner of the image.
    row 0 is top of image, row (height - 1) is bottom.
    """
    origin_x, origin_y, origin_yaw = origin
    # Grid cell relative to bottom-left
    grid_x = (col + 0.5) * resolution
    grid_y = (height - 1 - row + 0.5) * resolution

    if abs(origin_yaw) > 1e-6:
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        world_x = c * grid_x - s * grid_y + origin_x
        world_y = s * grid_x + c * grid_y + origin_y
    else:
        world_x = grid_x + origin_x
        world_y = grid_y + origin_y

    return world_x, world_y


def is_pair_dissimilar(new_pair: tuple, existing_pairs: list, thresh: float) -> bool:
    """Ensure the new pair is not too close to previously sampled pairs."""
    sx1, sy1, gx1, gy1 = new_pair
    for ex in existing_pairs:
        sx2, sy2, gx2, gy2 = ex
        # Direct similarity
        d_start = math.hypot(sx1 - sx2, sy1 - sy2)
        d_goal = math.hypot(gx1 - gx2, gy1 - gy2)
        if d_start < thresh and d_goal < thresh:
            return False
        # Reverse similarity (avoid sampling exact reverse route)
        d_rev_start = math.hypot(sx1 - gx2, sy1 - gy2)
        d_rev_goal = math.hypot(gx1 - sx2, gy1 - sy2)
        if d_rev_start < thresh and d_rev_goal < thresh:
            return False
    return True


def generate_waypoints(
    map_yaml: str,
    num_samples: int = 10,
    min_distance: float = 4.0,
    max_distance: float = 25.0,
    clearance_m: float = 0.6,
    similarity_thresh: float = 2.0,
    face_goal: bool = True,
    seed: int = None,
    preview_path: str = ''
) -> list:
    """Generate valid waypoint pairs."""
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    img, meta, res, origin, negate, occ_th, free_th = load_map(map_yaml)
    height, width = img.shape

    # 1. Compute occupancy probability
    if negate == 0:
        occ = (255.0 - img) / 255.0
    else:
        occ = img / 255.0

    # Strict free space
    free_mask = (occ < free_th).astype(np.uint8) * 255

    # 2. Erode free mask to enforce clearance from obstacles and unknown space
    clearance_px = max(1, int(math.ceil(clearance_m / res)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (clearance_px * 2 + 1, clearance_px * 2 + 1))
    eroded_mask = cv2.erode(free_mask, kernel)

    # 3. Identify connected components to guarantee reachability
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(eroded_mask)
    if num_labels <= 1:
        raise RuntimeError("No free space remaining after clearance erosion!")

    # Find largest component (excluding background label 0)
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest_label = int(np.argmax(areas) + 1)

    # Valid pixel indices in largest component
    valid_rows, valid_cols = np.where(labels == largest_label)
    num_valid = len(valid_rows)
    if num_valid < 2:
        raise RuntimeError("Not enough valid cells in the largest navigable component.")

    print(f"Map: {width}x{height} | Res: {res:.3f}m | Navigable cells: {num_valid} (clearance: {clearance_m}m)")

    # 4. Sample start-goal pairs
    sampled_pairs = []
    candidates = []
    max_attempts = num_samples * 1000
    attempts = 0

    while len(sampled_pairs) < num_samples and attempts < max_attempts:
        attempts += 1
        i_start = random.randint(0, num_valid - 1)
        i_goal = random.randint(0, num_valid - 1)
        if i_start == i_goal:
            continue

        r_s, c_s = valid_rows[i_start], valid_cols[i_start]
        r_g, c_g = valid_rows[i_goal], valid_cols[i_goal]

        sx, sy = grid_to_world(c_s, r_s, height, res, origin)
        gx, gy = grid_to_world(c_g, r_g, height, res, origin)

        dist = math.hypot(gx - sx, gy - sy)
        if dist < min_distance or dist > max_distance:
            continue

        pair_coords = (sx, sy, gx, gy)
        if not is_pair_dissimilar(pair_coords, candidates, similarity_thresh):
            continue

        # Valid pair found
        candidates.append(pair_coords)

        # Orientation calculation
        if face_goal:
            rad = math.atan2(gy - sy, gx - sx)
            deg = math.degrees(rad)
            start_yaw_rad = rad
            start_yaw_deg = deg
            goal_yaw_deg = deg
        else:
            deg = random.uniform(-180.0, 180.0)
            start_yaw_deg = deg
            start_yaw_rad = math.radians(deg)
            goal_yaw_deg = random.uniform(-180.0, 180.0)

        pair_dict = {
            'id': len(sampled_pairs),
            'start_x': round(sx, 3),
            'start_y': round(sy, 3),
            'start_yaw_rad': round(start_yaw_rad, 4),
            'start_yaw_deg': round(start_yaw_deg, 2),
            'goal_x': round(gx, 3),
            'goal_y': round(gy, 3),
            'goal_yaw_deg': round(goal_yaw_deg, 2),
            'distance': round(dist, 3),
            'start_pixel': (c_s, r_s),
            'goal_pixel': (c_g, r_g)
        }
        sampled_pairs.append(pair_dict)

    if len(sampled_pairs) < num_samples:
        print(f"Warning: Only generated {len(sampled_pairs)}/{num_samples} pairs after {attempts} attempts.")

    # 5. Optional preview visualization
    if preview_path:
        preview_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # Overlay clearance mask in faint cyan
        preview_rgb[eroded_mask > 0] = (
            preview_rgb[eroded_mask > 0] * 0.7 + np.array([180, 140, 0]) * 0.3
        ).astype(np.uint8)

        for p in sampled_pairs:
            cs, rs = p['start_pixel']
            cg, rg = p['goal_pixel']
            # Draw route arrow
            cv2.arrowedLine(preview_rgb, (cs, rs), (cg, rg), (0, 0, 230), 2, tipLength=0.03)
            # Draw start (green) and goal (blue)
            cv2.circle(preview_rgb, (cs, rs), 5, (0, 200, 0), -1)
            cv2.circle(preview_rgb, (cg, rg), 5, (230, 100, 0), -1)
            cv2.putText(preview_rgb, str(p['id']), (cs + 6, rs - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 150, 0), 1)

        os.makedirs(os.path.dirname(os.path.abspath(preview_path)) or '.', exist_ok=True)
        cv2.imwrite(preview_path, preview_rgb)
        print(f"Saved visual verification preview to: {preview_path}")

    return sampled_pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default_map = resolve_default_map()
    parser.add_argument('--map-yaml', default=default_map,
                        help=f'Path to map YAML file (default: {default_map})')
    parser.add_argument('-n', '--num-samples', type=int, default=10,
                        help='Number of waypoint pairs to generate (default: 10)')
    parser.add_argument('--min-dist', type=float, default=4.0,
                        help='Minimum distance in meters between start and goal (default: 4.0)')
    parser.add_argument('--max-dist', type=float, default=25.0,
                        help='Maximum distance in meters between start and goal (default: 25.0)')
    parser.add_argument('--clearance', type=float, default=0.6,
                        help='Robot clearance radius in meters from obstacles (default: 0.6)')
    parser.add_argument('--similarity-thresh', type=float, default=2.0,
                        help='Minimum distance in meters between distinct sampled pairs (default: 2.0)')
    parser.add_argument('--seed', type=int, default=None,
                        help='Optional random seed for reproducibility (default: None, non-deterministic)')
    parser.add_argument('--random-yaw', action='store_true',
                        help='Randomize start/goal yaw orientations instead of facing the goal')
    parser.add_argument('-o', '--output', default='data/waypoints.csv',
                        help='Output CSV filepath (default: data/waypoints.csv)')
    parser.add_argument('--preview', default='',
                        help='Optional filepath to save visual map overlay image (e.g. data/preview.png)')

    args = parser.parse_args()

    if not args.map_yaml:
        print("Error: --map-yaml must be specified (or clearpath_nav2_demos installed).", file=sys.stderr)
        sys.exit(1)

    print(f"Generating waypoints from {args.map_yaml}...")
    waypoints = generate_waypoints(
        map_yaml=args.map_yaml,
        num_samples=args.num_samples,
        min_distance=args.min_dist,
        max_distance=args.max_dist,
        clearance_m=args.clearance,
        similarity_thresh=args.similarity_thresh,
        face_goal=not args.random_yaw,
        seed=args.seed,
        preview_path=args.preview
    )

    # Save to CSV
    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    fields = ['id', 'start_x', 'start_y', 'start_yaw_rad', 'start_yaw_deg', 'goal_x', 'goal_y', 'goal_yaw_deg', 'distance']
    with open(args.output, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for wp in waypoints:
            row = {k: wp[k] for k in fields}
            writer.writerow(row)

    print(f"Successfully generated {len(waypoints)} waypoints to {args.output}")


if __name__ == '__main__':
    main()
