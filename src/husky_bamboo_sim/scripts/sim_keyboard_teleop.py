#!/usr/bin/env python3
"""Keyboard teleop for A300 Clearpath sim (TwistStamped on /{namespace}/cmd_vel)."""

from __future__ import annotations

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


HELP = """
Keyboard teleop (focus this terminal):
  w/s  : forward / backward
  a/d  : strafe left / right
  q/e  : rotate left / right
  space: stop
  x    : quit
"""


class SimKeyboardTeleop(Node):
  def __init__(self) -> None:
    super().__init__('sim_keyboard_teleop')
    self.declare_parameter('namespace', 'a300_00000')
    self.declare_parameter('linear_speed', 0.6)
    self.declare_parameter('angular_speed', 1.0)

    namespace = self.get_parameter('namespace').get_parameter_value().string_value.strip('/')
    self._linear = self.get_parameter('linear_speed').get_parameter_value().double_value
    self._angular = self.get_parameter('angular_speed').get_parameter_value().double_value
    topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'

    self._pub = self.create_publisher(TwistStamped, topic, 10)
    self.get_logger().info(f'Publishing TwistStamped on {topic}')
    print(HELP)

  def _publish(self, lx: float, ly: float, az: float) -> None:
    msg = TwistStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'base_link'
    msg.twist.linear.x = lx
    msg.twist.linear.y = ly
    msg.twist.angular.z = az
    self._pub.publish(msg)

  @staticmethod
  def _open_terminal():
    """Return a readable TTY stream, preferring stdin, else the controlling /dev/tty.

    ``ros2 launch`` gives child processes a pipe on stdin, so stdin is not a
    terminal there; /dev/tty still reaches the terminal the launch runs in.
    """
    if sys.stdin.isatty():
      return sys.stdin
    try:
      return open('/dev/tty', 'r', buffering=1, encoding='utf-8')
    except OSError:
      return None

  def run(self) -> None:
    stream = self._open_terminal()
    if stream is None:
      self.get_logger().warning(
        'No controlling terminal available; keyboard teleop disabled '
        '(launch with keyboard_teleop:=false to silence this)')
      return
    fd = stream.fileno()
    old = termios.tcgetattr(fd)
    try:
      tty.setcbreak(fd)
      while rclpy.ok():
        ch = stream.read(1)
        if ch == 'x':
          break
        if ch == ' ':
          self._publish(0.0, 0.0, 0.0)
        elif ch == 'w':
          self._publish(self._linear, 0.0, 0.0)
        elif ch == 's':
          self._publish(-self._linear, 0.0, 0.0)
        elif ch == 'a':
          self._publish(0.0, self._linear, 0.0)
        elif ch == 'd':
          self._publish(0.0, -self._linear, 0.0)
        elif ch == 'q':
          self._publish(0.0, 0.0, self._angular)
        elif ch == 'e':
          self._publish(0.0, 0.0, -self._angular)
        rclpy.spin_once(self, timeout_sec=0.0)
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old)
      self._publish(0.0, 0.0, 0.0)
      if stream is not sys.stdin:
        stream.close()


def main() -> None:
  rclpy.init()
  node = SimKeyboardTeleop()
  try:
    node.run()
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
