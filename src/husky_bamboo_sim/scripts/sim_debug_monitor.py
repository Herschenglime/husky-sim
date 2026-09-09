#!/usr/bin/env python3
"""Periodic health monitor for the A300 Clearpath Gazebo simulation."""

from __future__ import annotations

import subprocess

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimDebugMonitor(Node):
  def __init__(self) -> None:
    super().__init__('sim_debug_monitor')
    self.declare_parameter('namespace', 'a300_00000')
    self.declare_parameter('world', 'warehouse')
    self.declare_parameter('check_interval_sec', 5.0)

    self._namespace = self.get_parameter('namespace').get_parameter_value().string_value.strip('/')
    self._world = self.get_parameter('world').get_parameter_value().string_value
    interval = self.get_parameter('check_interval_sec').get_parameter_value().double_value

    self._checks: dict[str, bool] = {}
    self._timer = self.create_timer(interval, self._run_checks)

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    ns = f'/{self._namespace}' if self._namespace else ''
    self._odom_sub = self.create_subscription(
        Odometry,
        f'{ns}/platform/odom',
        lambda _msg: self._mark('odom', True),
        10,
    )
    self._clock_sub = self.create_subscription(
        Clock,
        '/clock',
        lambda _msg: self._mark('clock', True),
        qos,
    )

    self.get_logger().info(
        f'Monitoring A300 sim namespace={self._namespace or "/"} world={self._world}'
    )

  def _mark(self, name: str, ok: bool) -> None:
    self._checks[name] = ok

  def _topic_exists(self, topic: str) -> bool:
    names = {name for name, _types in self.get_topic_names_and_types()}
    return topic in names

  def _node_running(self, pattern: str) -> bool:
    try:
      out = subprocess.run(
          ['pgrep', '-f', pattern],
          capture_output=True,
          text=True,
          check=False,
      )
      return out.returncode == 0
    except OSError:
      return False

  def _run_checks(self) -> None:
    ns = f'/{self._namespace}' if self._namespace else ''
    results = {
        'gz_sim': self._node_running('gz sim'),
        'clock': self._checks.get('clock', False) or self._topic_exists('/clock'),
        'robot_description': self._topic_exists(f'{ns}/robot_description'),
        'cmd_vel': self._topic_exists(f'{ns}/cmd_vel'),
        'platform_odom': self._checks.get('odom', False) or self._topic_exists(f'{ns}/platform/odom'),
        'lidar2d': self._topic_exists(f'{ns}/sensors/lidar2d_0/scan'),
        'imu': self._topic_exists(f'{ns}/sensors/imu_0/data'),
    }

    ok_count = sum(1 for v in results.values() if v)
    total = len(results)
    status = 'HEALTHY' if ok_count == total else 'DEGRADED' if ok_count >= total - 2 else 'UNHEALTHY'

    lines = [f'[{status}] A300 sim health ({ok_count}/{total}) world={self._world}']
    for name, ok in results.items():
      lines.append(f'  {"OK" if ok else "FAIL":4s} {name}')
    self.get_logger().info('\n'.join(lines))


def main() -> None:
  rclpy.init()
  node = SimDebugMonitor()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
