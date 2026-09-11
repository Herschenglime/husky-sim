import os
import sys
import json
import tempfile
import pytest

# Ensure ros2_ws/src/nav_worlds/scripts is on sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))

import rclpy
from rclpy.parameter import Parameter
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from log_state import StateLogger


@pytest.fixture(scope="module")
def ros_context():
    if 'ROS_LOG_DIR' not in os.environ:
        os.environ['ROS_LOG_DIR'] = os.path.join(tempfile.gettempdir(), 'ros_log')
    os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_state_logger_lifecycle_transitions(ros_context):
    with tempfile.TemporaryDirectory() as tmpdir:
        out_file1 = os.path.join(tmpdir, "run_000", "state.jsonl")
        out_file2 = os.path.join(tmpdir, "run_001", "state.jsonl")

        node = StateLogger(
            node_name="test_state_logger",
            output_file=out_file1,
            target_frame="map",
            base_frame="base_link",
            odom_topic="/test/odom",
            scan_topic="/test/scan",
            cmd_vel_topic="/test/cmd_vel"
        )

        # 1. Initially Unconfigured
        assert node.current_state == 'unconfigured'
        assert node.file is None
        assert not node._is_active
        assert node.target_frame == "map"
        assert node.base_frame == "base_link"

        # 2. Configure -> Inactive
        ret = node.trigger_configure()
        assert ret == TransitionCallbackReturn.SUCCESS
        assert node.current_state == 'inactive'
        assert node.file is None
        assert not os.path.exists(out_file1)

        # 3. Simulate scan while Inactive (should NOT write anything)
        scan = LaserScan()
        scan.ranges = [1.0, 2.0, 3.0]
        node.scan_cb(scan)
        assert not os.path.exists(out_file1)

        # 4. Activate -> Active
        ret = node.trigger_activate()
        assert ret == TransitionCallbackReturn.SUCCESS
        assert node.current_state == 'active'
        assert node.file is not None
        assert os.path.exists(out_file1)

        # 5. Verify parameter updates are REJECTED while Active
        results = node.set_parameters([Parameter('output', Parameter.Type.STRING, out_file2)])
        assert not results[0].successful
        assert node.output_file == out_file1

        # 6. Provide Odom and Scan data while Active
        odom = Odometry()
        odom.pose.pose.position.x = 2.5
        odom.pose.pose.position.y = 3.5
        odom.pose.pose.orientation.w = 1.0
        node.odom_cb(odom)
        node.scan_cb(scan)

        # 7. Deactivate -> Inactive (flushes and closes file)
        ret = node.trigger_deactivate()
        assert ret == TransitionCallbackReturn.SUCCESS
        assert node.current_state == 'inactive'
        assert node.file is None

        # Verify content of out_file1 (both odom and resolved x, y coordinates)
        assert os.path.exists(out_file1)
        with open(out_file1, 'r') as f:
            lines = [json.loads(line) for line in f if line.strip()]
        assert len(lines) == 1
        assert lines[0]['x'] == 2.5
        assert lines[0]['y'] == 3.5
        assert lines[0]['odom_x'] == 2.5
        assert lines[0]['odom_y'] == 3.5
        assert lines[0]['scan'] == [1.0, 2.0, 3.0]

        # 8. Update output parameter while Inactive for next run (should SUCCEED)
        results = node.set_parameters([Parameter('output', Parameter.Type.STRING, out_file2)])
        assert results[0].successful
        assert node.output_file == out_file2

        # 9. Activate second run -> Active
        ret = node.trigger_activate()
        assert ret == TransitionCallbackReturn.SUCCESS
        assert node.file is not None

        odom.pose.pose.position.x = 5.0
        odom.pose.pose.position.y = 7.0
        node.odom_cb(odom)
        node.scan_cb(scan)

        ret = node.trigger_deactivate()
        assert ret == TransitionCallbackReturn.SUCCESS
        assert node.file is None

        # Verify content of out_file2
        with open(out_file2, 'r') as f:
            lines2 = [json.loads(line) for line in f if line.strip()]
        assert len(lines2) == 1
        assert lines2[0]['x'] == 5.0
        assert lines2[0]['y'] == 7.0

        # 10. Cleanup -> Unconfigured
        ret = node.trigger_cleanup()
        assert ret == TransitionCallbackReturn.SUCCESS
        assert node.current_state == 'unconfigured'

        # 11. Shutdown -> Finalized
        ret = node.trigger_shutdown()
        assert ret == TransitionCallbackReturn.SUCCESS
        assert node.current_state == 'finalized'

        node.destroy_node()
