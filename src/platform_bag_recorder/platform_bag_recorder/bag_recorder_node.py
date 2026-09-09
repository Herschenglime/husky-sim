#!/usr/bin/env python3
"""Record a platform's sensors to a rosbag2 bag.

Follows turtlebot4_bag_recorder: message types and publisher QoS are both
discovered at runtime, so best-effort sensor topics are actually received and
the same node works whatever the message types happen to be.

What it adds is that the topic set is not fixed. Each recorded role - the 2D
scan, the camera, the IMU - is a *slot* with candidate topics, and the first
candidate actually being published wins. Slots may be optional. That is what
lets one profile cover an a200 with a camera and 3D lidar and an a200 carrying
only the 2D lidar, without the caller knowing which is which: the navigation
robot config drops the sensor arch, and the camera and 3D lidar with it.

    ros2 run platform_bag_recorder bag_recorder --ros-args -p platform:=go1
"""

import os
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.serialization import serialize_message
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool

import rosbag2_py

from platform_bag_recorder import profiles


class BagRecorderNode(Node):

    def __init__(self):
        super().__init__('bag_recorder_node')

        self.declare_parameter('platform', 'a200')      # a200 | go1
        self.declare_parameter('namespace', '')         # '' -> profile default
        self.declare_parameter('bag_base_dir', os.path.expanduser('~/bags'))
        self.declare_parameter('bag_name', '')          # '' -> timestamped
        self.declare_parameter('storage_id', 'mcap')
        self.declare_parameter('discovery_period_s', 2.0)
        # How long to keep looking for optional topics before settling on the
        # set to record. Without a deadline a missing optional sensor would
        # hold up recording for ever.
        self.declare_parameter('optional_grace_s', 15.0)
        # 0.0 records every message as it arrives. Set >0 to write all topics
        # synchronized at one uniform rate instead.
        self.declare_parameter('sync_rate_hz', 0.0)
        self.declare_parameter('stop_topic', '')

        platform = str(self.get_parameter('platform').value)
        namespace = str(self.get_parameter('namespace').value)
        try:
            self.slots = profiles.build(platform, namespace)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            raise

        # Expand ~ here rather than relying on the parameter default. A launch
        # argument or a command line always arrives as a literal string - the
        # shell does not expand it inside `bag_base_dir:=~/bags` - and rosbag2
        # will happily create a directory *named* "~" under the working
        # directory, so the bag is written and the path in the log looks right
        # while the bag is nowhere near where it was asked to go.
        base_dir = os.path.expanduser(str(self.get_parameter('bag_base_dir').value))
        bag_name = self.get_parameter('bag_name').value or (
            f'{platform}_' + datetime.now().strftime('%Y_%m_%d-%H_%M_%S'))
        self.bag_uri = os.path.abspath(os.path.join(base_dir, bag_name))
        os.makedirs(base_dir, exist_ok=True)

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(
                uri=self.bag_uri,
                storage_id=str(self.get_parameter('storage_id').value)),
            rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                        output_serialization_format='cdr'))

        self._subscribed = {}      # topic -> subscription (probe, then recording)
        self._received = {}        # topic -> messages seen, decides the winner
        self._types = {}           # topic -> message type name
        self._probed_last = {}     # topic -> newest message seen while probing
        self._resolved = {}        # slot name -> topic
        self._latest = {}          # topic -> most recent msg (sync mode)
        self._msg_count = 0
        self._started = None
        self._settled = False
        self._sync_timer = None
        self._stop_requested = False

        stop_topic = str(self.get_parameter('stop_topic').value)
        if stop_topic:
            self.create_subscription(
                Bool, stop_topic, self._stop_callback,
                QoSProfile(depth=1,
                           reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
            self.get_logger().info(f'Will stop when {stop_topic} becomes true')

        self.get_logger().info(f'Platform {platform}, recording to {self.bag_uri}')
        self._started = self.get_clock().now()
        period = float(self.get_parameter('discovery_period_s').value)
        self._discovery_timer = self.create_timer(period, self._discover)
        self._discover()

    # -- discovery ----------------------------------------------------------

    def _discover(self):
        """Subscribe to every advertised candidate; choose winners at settle.

        A topic being advertised does not mean it carries data. The go1's
        camera advertises /color/image_raw/compressed with a publisher, and it
        emits nothing - image_transport advertises the transport whether or not
        anything drives it. Preferring it on the strength of the advertisement
        alone put an empty topic in the bag while a perfectly good 10 Hz raw
        stream sat next to it. So probe first: subscribe to the candidates,
        see which actually deliver, and only then pick.
        """
        available = dict(self.get_topic_names_and_types())

        for slot in self.slots:
            for topic in slot.candidates:
                if topic in self._subscribed:
                    continue
                if topic in available and available[topic]:
                    self._probe(topic, available[topic][0])

        elapsed = (self.get_clock().now() - self._started).nanoseconds * 1e-9
        grace = float(self.get_parameter('optional_grace_s').value)
        if self._settled:
            return

        # A slot is satisfiable once any of its candidates has delivered.
        unsatisfied = [
            s.name for s in self.slots if s.required
            and not any((self._received.get(c) or (not s.probe and c in self._types))
                        for c in s.candidates)]
        if unsatisfied:
            if elapsed > grace * 2:
                self.get_logger().warn(f'still waiting for: {unsatisfied}')
            return
        if elapsed < grace:
            return
        self._settle()

    def _settle(self):
        for slot in self.slots:
            for topic in slot.candidates:
                seen = self._received.get(topic)
                advertised = topic in self._types
                if seen or (not slot.probe and advertised):
                    self._resolved[slot.name] = topic
                    self.writer.create_topic(self._topic_metadata(
                        len(self._resolved), topic, self._types[topic]))
                    note = ''
                    if topic != slot.candidates[0]:
                        dead = [c for c in slot.candidates[:slot.candidates.index(topic)]
                                if c in self._received]
                        if dead:
                            note = f'  (preferred {dead[0]} is advertised but silent)'
                    self.get_logger().info(
                        f'  {slot.name:10} -> {topic} [{self._types[topic]}]{note}')
                    break
        # Drop probes we are not recording, so they cost nothing.
        for topic, sub in list(self._subscribed.items()):
            if topic not in self._resolved.values():
                self.destroy_subscription(sub)
                del self._subscribed[topic]
        self._settled = True

        # Flush what was captured while probing, so one-shot latched messages
        # are in the bag rather than merely having been seen.
        stamp = self.get_clock().now().nanoseconds
        for topic in self._resolved.values():
            msg = self._probed_last.get(topic)
            if msg is not None:
                self.writer.write(topic, serialize_message(msg), stamp)
                self._msg_count += 1
        self._probed_last.clear()

        skipped = [s.name for s in self.slots if s.name not in self._resolved]
        self.get_logger().info(
            f'Recording {len(self._resolved)} topics: {sorted(self._resolved)}')
        if skipped:
            self.get_logger().info(
                f'Not published on this robot, skipped: {sorted(skipped)}')
        rate = float(self.get_parameter('sync_rate_hz').value)
        if rate > 0.0:
            self.get_logger().info(f'Writing all topics synchronized at {rate:.2f} Hz')
            self._sync_timer = self.create_timer(1.0 / rate, self._write_snapshot)
        self._discovery_timer.cancel()

    def _probe(self, topic, type_name):
        try:
            msg_type = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.get_logger().error(f'Cannot import {type_name} for {topic}: {exc}')
            return False
        self._types[topic] = type_name
        self._received[topic] = 0
        self._subscribed[topic] = self.create_subscription(
            msg_type, topic, lambda msg, t=topic: self._callback(msg, t),
            self._matching_qos(topic))
        return True

    @staticmethod
    def _topic_metadata(topic_id, name, type_name):
        """TopicMetadata gained a required 'id' between Humble and Jazzy."""
        try:
            return rosbag2_py.TopicMetadata(
                id=topic_id, name=name, type=type_name,
                serialization_format='cdr')
        except TypeError:
            return rosbag2_py.TopicMetadata(
                name=name, type=type_name, serialization_format='cdr')

    def _matching_qos(self, topic):
        """Match the publishers, or best-effort sensor data is never received.

        With several publishers on one topic - /tf commonly has the EKF, AMCL
        and a transient-local static publisher - take the weakest setting
        across all of them, since a subscription only matches publishers
        offering at least what it asks for.
        """
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=50,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        infos = self.get_publishers_info_by_topic(topic)
        if infos:
            ps = [i.qos_profile for i in infos]
            if any(p.reliability == QoSReliabilityPolicy.BEST_EFFORT for p in ps):
                qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            if all(p.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL for p in ps):
                qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos

    # -- writing ------------------------------------------------------------

    def _callback(self, msg, topic):
        self._received[topic] = self._received.get(topic, 0) + 1
        if not self._settled:
            # Hold the newest probed message. Latched (TRANSIENT_LOCAL) topics
            # such as /tf_static deliver once, on subscribe - which happens
            # during probing - and never again, so without this the winner is
            # chosen on the strength of a message that then never reaches the
            # bag and /tf_static lands in it with a count of zero. A bag with
            # no static transforms cannot be replayed into a working TF tree.
            self._probed_last[topic] = msg
            return
        if topic not in self._resolved.values():
            return
        if self._sync_timer is not None:
            self._latest[topic] = msg      # snapshot mode writes on the timer
            return
        self.writer.write(topic, serialize_message(msg),
                          self.get_clock().now().nanoseconds)
        self._msg_count += 1
        if self._msg_count % 1000 == 0:
            self.get_logger().info(f'{self._msg_count} messages written')

    def _write_snapshot(self):
        if len(self._latest) < len(self._resolved):
            return
        stamp = self.get_clock().now().nanoseconds
        for topic in self._resolved.values():
            self.writer.write(topic, serialize_message(self._latest[topic]), stamp)
            self._msg_count += 1

    def _stop_callback(self, msg):
        if msg.data and not self._stop_requested:
            self._stop_requested = True
            self.get_logger().info('Stop requested; finalizing bag.')
            rclpy.shutdown()

    def close(self):
        self.get_logger().info(
            f'Closing bag ({self._msg_count} messages): {self.bag_uri}')
        del self.writer     # SequentialWriter finalizes on destruction


def main(args=None):
    # Let Ctrl+C raise KeyboardInterrupt rather than having rclpy tear the
    # context down mid-flight, so subscriptions and the DDS participant are
    # destroyed cleanly and the bag is finalized.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.close()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
