import os
import sys
import tempfile
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import rosbag2_py
from std_msgs.msg import String

# Ensure scripts directory is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))

from bag_recorder import RosbagRecorder  # noqa: E402


@pytest.fixture(scope='module')
def ros_context():
    if 'ROS_LOG_DIR' not in os.environ:
        os.environ['ROS_LOG_DIR'] = os.path.join(tempfile.gettempdir(), 'ros_log')
    os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class MessagePublisher(Node):
    def __init__(self, node_name='test_msg_pub'):
        super().__init__(node_name)
        self.pub_a = self.create_publisher(String, '/test_bag/topic_a', 10)
        self.pub_b = self.create_publisher(String, '/test_bag/topic_b', 10)

    def publish_a(self, text='hello_a'):
        msg = String()
        msg.data = text
        self.pub_a.publish(msg)

    def publish_b(self, text='hello_b'):
        msg = String()
        msg.data = text
        self.pub_b.publish(msg)


def test_rosbag_recorder_mcap_format(ros_context):
    pub_node = MessagePublisher('pub_mcap_test')
    executor = SingleThreadedExecutor()
    executor.add_node(pub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    with tempfile.TemporaryDirectory() as tmpdir:
        bag_dir = os.path.join(tmpdir, 'test_mcap_bag')
        recorder = RosbagRecorder(
            output_uri=bag_dir,
            topics=['/test_bag/topic_a'],
            storage_id='mcap',
            use_sim_time=False,
            node_name='rec_mcap_test',
        )

        recorder.start()
        assert recorder.is_recording is True

        # Publish test messages
        for i in range(5):
            pub_node.publish_a(f'msg_{i}')
            time.sleep(0.05)

        recorder.stop()
        assert recorder.is_recording is False

        # Verify metadata
        info = rosbag2_py.Info()
        meta = info.read_metadata(bag_dir, 'mcap')
        assert meta.storage_identifier == 'mcap'
        assert len(meta.files) >= 1
        assert meta.files[0].path.endswith('.mcap')
        assert meta.message_count >= 1

    executor.shutdown()
    thread.join(timeout=1.0)
    pub_node.destroy_node()


def test_rosbag_recorder_topic_filtering(ros_context):
    pub_node = MessagePublisher('pub_filter_test')
    executor = SingleThreadedExecutor()
    executor.add_node(pub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    with tempfile.TemporaryDirectory() as tmpdir:
        bag_dir = os.path.join(tmpdir, 'test_filter_bag')
        recorder = RosbagRecorder(
            output_uri=bag_dir,
            topics=['/test_bag/topic_a'],
            storage_id='mcap',
            use_sim_time=False,
            node_name='rec_filter_test',
        )

        with recorder:
            for _ in range(5):
                pub_node.publish_a('included')
                pub_node.publish_b('excluded')
                time.sleep(0.05)

        info = rosbag2_py.Info()
        meta = info.read_metadata(bag_dir, 'mcap')
        recorded_topics = [t.topic_metadata.name for t in meta.topics_with_message_count]
        assert '/test_bag/topic_a' in recorded_topics
        assert '/test_bag/topic_b' not in recorded_topics

    executor.shutdown()
    thread.join(timeout=1.0)
    pub_node.destroy_node()


def test_rosbag_recorder_context_manager(ros_context):
    with tempfile.TemporaryDirectory() as tmpdir:
        bag_dir = os.path.join(tmpdir, 'test_ctx_bag')
        recorder = RosbagRecorder(
            output_uri=bag_dir,
            topics=['/test_bag/topic_a'],
            storage_id='mcap',
            use_sim_time=False,
            node_name='rec_ctx_test',
        )

        assert recorder.is_recording is False
        with recorder:
            assert recorder.is_recording is True
            time.sleep(0.1)

        assert recorder.is_recording is False
        assert os.path.exists(os.path.join(bag_dir, 'metadata.yaml'))


def test_rosbag_recorder_sequential_runs(ros_context):
    pub_node = MessagePublisher('pub_seq_test')
    executor = SingleThreadedExecutor()
    executor.add_node(pub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    with tempfile.TemporaryDirectory() as tmpdir:
        for i in range(2):
            bag_dir = os.path.join(tmpdir, f'run_{i:03d}', 'bag')
            recorder = RosbagRecorder(
                output_uri=bag_dir,
                topics=['/test_bag/topic_a'],
                storage_id='mcap',
                use_sim_time=False,
                node_name=f'rec_seq_{i}',
            )

            with recorder:
                for j in range(3):
                    pub_node.publish_a(f'run_{i}_msg_{j}')
                    time.sleep(0.05)

            assert os.path.exists(os.path.join(bag_dir, 'metadata.yaml'))
            info = rosbag2_py.Info()
            meta = info.read_metadata(bag_dir, 'mcap')
            assert meta.storage_identifier == 'mcap'
            assert meta.message_count >= 1

    executor.shutdown()
    thread.join(timeout=1.0)
    pub_node.destroy_node()
