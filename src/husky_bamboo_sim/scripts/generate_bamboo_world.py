#!/usr/bin/env python3
"""Generate a Gazebo Harmonic SDF world with bamboo culm obstacles."""

import argparse
import random
import sys
from pathlib import Path


def make_culm(name: str, x: float, y: float, radius: float, height: float) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {height / 2.0} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>
          </geometry>
          <material>
            <diffuse>0.35 0.55 0.20 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def generate_world(
    output_path: Path,
    field_size: float = 80.0,
    row_spacing: float = 2.5,
    culm_spacing: float = 0.9,
    row_gap_variance: float = 0.4,
    culm_gap_variance: float = 0.35,
    min_radius: float = 0.04,
    max_radius: float = 0.09,
    min_height: float = 4.0,
    max_height: float = 8.0,
    skip_probability: float = 0.18,
    seed: int = 42,
) -> None:
    random.seed(seed)
    half = field_size / 2.0
    culms = []
    culm_idx = 0
    y = -half + 3.0
    while y < half - 3.0:
        x = -half + 3.0
        row_offset = random.uniform(-row_gap_variance, row_gap_variance)
        while x < half - 3.0:
            if random.random() > skip_probability:
                radius = random.uniform(min_radius, max_radius)
                height = random.uniform(min_height, max_height)
                px = x + random.uniform(-culm_gap_variance, culm_gap_variance)
                py = y + row_offset + random.uniform(-culm_gap_variance, culm_gap_variance)
                culms.append(make_culm(f'bamboo_{culm_idx}', px, py, radius, height))
                culm_idx += 1
            x += culm_spacing + random.uniform(-0.2, 0.2)
        y += row_spacing + random.uniform(-0.3, 0.3)

    culm_models = '\n'.join(culms)
    world = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="bamboo_field">
    <physics name="default" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0.6 0.8</pose>
      <diffuse>0.95 0.95 0.88 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{field_size} {field_size}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{field_size} {field_size}</size>
            </plane>
          </geometry>
          <material>
            <diffuse>0.45 0.42 0.30 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    {culm_models}
  </world>
</sdf>
"""
    output_path.write_text(world)
    print(f'Wrote {culm_idx} bamboo culms to {output_path}')


def main() -> int:
    parser = argparse.ArgumentParser(description='Generate bamboo field Gazebo world')
    parser.add_argument(
        '-o', '--output',
        type=Path,
        default=Path(__file__).resolve().parent.parent / 'worlds' / 'bamboo_field.sdf',
    )
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    generate_world(args.output, seed=args.seed)
    return 0


if __name__ == '__main__':
    sys.exit(main())
