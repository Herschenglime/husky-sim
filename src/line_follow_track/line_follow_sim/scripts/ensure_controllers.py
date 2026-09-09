#!/usr/bin/env python3
"""Activate the a200's ros2_control controllers, but only the ones nobody else did.

Whether clearpath_control spawns the controllers itself depends on the release:

  * clearpath-simulator <= 2.9.3 spawned nothing under sim - its loop only
    picked up controllers whose name contains 'controller' and contains neither
    'manager' nor 'platform', and both of the a200's controllers fail that
    filter. controller_manager came up empty and the robot ignored cmd_vel.
  * Newer releases spawn both correctly.

Spawning them unconditionally therefore breaks on the newer release: the second
spawner finds them already active and exits 1 with "can not be configured from
'active' state". Spawning them not at all breaks on the older one. So ask the
controller_manager what is actually running and fill in only the gaps.

    ensure_controllers.py -c /a200_0000/controller_manager [--settle 20]
                          [--timeout 60] joint_state_broadcaster ...
"""

import argparse
import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


def active_controllers(node, client, timeout):
    """Names of the controllers currently in 'active' state, or None if unreachable."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        return None
    return {c.name for c in future.result().controller if c.state == 'active'}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('-c', '--controller-manager', required=True)
    ap.add_argument('--timeout', type=float, default=60.0,
                    help='seconds to wait for controller_manager to exist')
    ap.add_argument('--settle', type=float, default=20.0,
                    help="seconds to give someone else's spawner to finish first")
    ap.add_argument('controllers', nargs='+')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = rclpy.create_node('ensure_controllers')
    client = node.create_client(ListControllers, f'{args.controller_manager}/list_controllers')

    # Poll rather than checking once: at the moment we first look, the release
    # that does spawn its own controllers may still be part-way through doing
    # it, and we would race in behind it and collide anyway.
    deadline = time.monotonic() + args.settle
    wanted = set(args.controllers)
    missing = wanted
    while True:
        active = active_controllers(node, client, args.timeout)
        if active is None:
            print('ensure_controllers: no controller_manager at '
                  f'{args.controller_manager} after {args.timeout:.0f}s', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1
        missing = wanted - active
        if not missing or time.monotonic() >= deadline:
            break
        time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()

    if not missing:
        print(f'ensure_controllers: already active: {", ".join(sorted(wanted))}')
        return 0

    print(f'ensure_controllers: spawning {", ".join(sorted(missing))}')
    return subprocess.call([
        'ros2', 'run', 'controller_manager', 'spawner',
        '--controller-manager-timeout', str(int(args.timeout)),
        '-c', args.controller_manager, *sorted(missing),
    ])


if __name__ == '__main__':
    sys.exit(main())
