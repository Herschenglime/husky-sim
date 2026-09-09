#!/usr/bin/env python3
"""
Bag recorder node for TurtleBot4.

Records a configurable list of topics to a rosbag2 bag. Topic message types
and publisher QoS profiles are discovered at runtime, so the same node works
on ROS 2 Humble (physical TurtleBot4) and ROS 2 Jazzy (gz_bringup simulation)
even when message types differ between distros (e.g. /cmd_vel being
geometry_msgs/Twist on Humble vs geometry_msgs/TwistStamped in newer sims).
"""

import os
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool

import rosbag2_py


class BagRecorderNode(Node):

    def __init__(self):
        super().__init__('bag_recorder_node')

        self.declare_parameter('topics', [
            '/scan',
            '/tf',
            '/odom',
            '/oakd/rgb/preview/image_raw',
            '/cmd_vel',
        ])
        self.declare_parameter('bag_base_dir', os.path.expanduser('~/bags'))
        self.declare_parameter('bag_name', '')          # empty -> timestamped
        self.declare_parameter('storage_id', 'sqlite3') # 'sqlite3' or 'mcap'
        self.declare_parameter('discovery_period_s', 2.0)
        # Synchronized recording: 0.0 = auto (measure and use the slowest
        # publisher's rate), otherwise record all topics at this fixed rate.
        self.declare_parameter('sync_rate_hz', 0.0)
        self.declare_parameter('rate_measure_window_s', 5.0)
        self.declare_parameter('stop_topic', '')

        self.topics = list(self.get_parameter('topics').value)
        base_dir = self.get_parameter('bag_base_dir').value
        bag_name = self.get_parameter('bag_name').value
        storage_id = self.get_parameter('storage_id').value

        if not bag_name:
            bag_name = 'tb4_' + datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
        self.bag_uri = os.path.join(base_dir, bag_name)
        os.makedirs(base_dir, exist_ok=True)

        # --- open the bag writer ---
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=self.bag_uri, storage_id=storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            ),
        )

        self._subscribed = {}   # topic -> subscription
        self._msg_count = 0

        # --- synchronization state ---
        # Latest message per topic; snapshots of this dict are written to the
        # bag at a single uniform rate so all topics stay in lockstep.
        self._latest = {}        # topic -> most recent msg
        self._arrivals = {}      # topic -> msg count (rate measurement)
        self._measure_start = None
        self._sync_timer = None
        self._stop_requested = False

        stop_topic = str(self.get_parameter('stop_topic').value)
        self._stop_subscription = None
        if stop_topic:
            stop_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._stop_subscription = self.create_subscription(
                Bool, stop_topic, self._stop_callback, stop_qos)
            self.get_logger().info(
                f'Will stop recording when {stop_topic} becomes true')

        self.get_logger().info(f'Recording to: {self.bag_uri}')
        self.get_logger().info(f'Waiting for topics: {self.topics}')

        # Periodically try to subscribe to topics that are not yet available
        # (handles nodes/sim starting after the recorder).
        period = float(self.get_parameter('discovery_period_s').value)
        self._discovery_timer = self.create_timer(period, self._discover_topics)
        self._discover_topics()

    # ------------------------------------------------------------------ #

    def _discover_topics(self):
        if len(self._subscribed) == len(self.topics):
            # All topics found: keep this timer running until the sync
            # timer has been started (it also drives rate measurement).
            if self._maybe_start_sync():
                self._discovery_timer.cancel()
            return

        available = dict(self.get_topic_names_and_types())

        for topic in self.topics:
            if topic in self._subscribed:
                continue
            if topic not in available or not available[topic]:
                continue

            type_name = available[topic][0]
            try:
                msg_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as e:
                self.get_logger().error(
                    f'Cannot import type {type_name} for {topic}: {e}')
                continue

            qos = self._matching_qos(topic)

            # Register the topic with the bag
            self.writer.create_topic(
                self._make_topic_metadata(len(self._subscribed), topic,
                                          type_name))

            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self._callback(msg, t),
                qos,
            )
            self._subscribed[topic] = sub
            self.get_logger().info(
                f'Subscribed to {topic} [{type_name}] '
                f'(reliability={qos.reliability.name})')

    @staticmethod
    def _make_topic_metadata(topic_id, name, type_name):
        """TopicMetadata gained a required 'id' arg between Humble and
        Jazzy. Try the Jazzy signature first, fall back to Humble's."""
        try:
            return rosbag2_py.TopicMetadata(
                id=topic_id,
                name=name,
                type=type_name,
                serialization_format='cdr',
            )
        except TypeError:
            return rosbag2_py.TopicMetadata(
                name=name,
                type=type_name,
                serialization_format='cdr',
            )

    def _matching_qos(self, topic):
        """Match the publisher's QoS so best-effort sensor topics
        (e.g. /scan, camera images) are actually received."""
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        infos = self.get_publishers_info_by_topic(topic)
        if infos:
            # A subscription only matches publishers offering at least its QoS, so
            # with several publishers (e.g. /tf on Clearpath robots: ekf, amcl and a
            # TRANSIENT_LOCAL controller) take the weakest setting across all of them.
            profiles = [info.qos_profile for info in infos]
            if any(p.reliability == QoSReliabilityPolicy.BEST_EFFORT for p in profiles):
                qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            if all(p.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL for p in profiles):
                qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos

    def _callback(self, msg, topic):
        # Cache only; writing happens at a uniform rate in _write_snapshot()
        # so every topic is recorded synchronized at the same frequency.
        self._latest[topic] = msg
        self._arrivals[topic] = self._arrivals.get(topic, 0) + 1

    def _stop_callback(self, msg):
        if not msg.data or self._stop_requested:
            return
        self._stop_requested = True
        self.get_logger().info('Goal reached; stopping and finalizing bag.')
        rclpy.shutdown()

    def _maybe_start_sync(self):
        """Start the uniform-rate snapshot timer. Returns True once started.

        If sync_rate_hz > 0 it is used directly; otherwise incoming message
        rates are measured for rate_measure_window_s and the slowest
        publisher's rate is used.
        """
        if self._sync_timer is not None:
            return True

        rate = float(self.get_parameter('sync_rate_hz').value)

        if rate <= 0.0:
            now = self.get_clock().now()
            if self._measure_start is None:
                self._measure_start = now
                self._arrivals = {t: 0 for t in self.topics}
                self.get_logger().info('Measuring topic rates to find the '
                                       'slowest publisher...')
                return False
            window = float(
                self.get_parameter('rate_measure_window_s').value)
            elapsed = (now - self._measure_start).nanoseconds * 1e-9
            if elapsed < window:
                return False
            rates = {t: self._arrivals.get(t, 0) / elapsed
                     for t in self.topics}
            for t, r in rates.items():
                self.get_logger().info(f'  {t}: {r:.2f} Hz')
            slowest_topic = min(rates, key=rates.get)
            rate = rates[slowest_topic]
            if rate <= 0.0:
                self.get_logger().warn(
                    f'{slowest_topic} published nothing during measurement '
                    f'(e.g. /cmd_vel while idle); falling back to 1.0 Hz. '
                    f'Set sync_rate_hz to override.')
                rate = 1.0
            else:
                self.get_logger().info(
                    f'Slowest publisher: {slowest_topic} ({rate:.2f} Hz)')

        self.get_logger().info(f'Recording all topics at {rate:.2f} Hz')
        self._sync_timer = self.create_timer(1.0 / rate, self._write_snapshot)
        return True

    def _write_snapshot(self):
        # Wait until every topic has produced at least one message so the
        # bag contains complete, aligned samples from the very first row.
        if len(self._latest) < len(self.topics):
            return
        stamp = self.get_clock().now().nanoseconds
        for topic in self.topics:
            self.writer.write(topic, serialize_message(self._latest[topic]),
                              stamp)
            self._msg_count += 1
        if self._msg_count % 500 < len(self.topics):
            self.get_logger().info(f'{self._msg_count} messages written')

    def close(self):
        self.get_logger().info(
            f'Closing bag ({self._msg_count} messages): {self.bag_uri}')
        del self.writer  # SequentialWriter finalizes on destruction


def main(args=None):
    # SignalHandlerOptions.NO: let Ctrl+C raise KeyboardInterrupt instead of
    # having rclpy shut the context down mid-flight. This keeps the context
    # valid during cleanup so the node/subscriptions and the DDS participant
    # are destroyed cleanly (an unclean exit can leave stale endpoints on the
    # robot's publishers and break topic matching on the next launch).
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
