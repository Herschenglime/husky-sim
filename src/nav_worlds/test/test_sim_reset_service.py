import os
import sys
import math
import tempfile
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))

import rclpy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from send_goal import publish_goal_marker


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


def test_set_entity_pose_request_structure(ros_context):
    node = rclpy.create_node('test_teleport_service')
    req = SetEntityPose.Request()
    req.entity.name = "a200_0000/robot"
    req.entity.type = Entity.MODEL
    req.pose.position.x = 10.0
    req.pose.position.y = -5.0
    req.pose.position.z = 0.3
    yaw_rad = math.pi / 2.0
    req.pose.orientation.z = math.sin(yaw_rad / 2.0)
    req.pose.orientation.w = math.cos(yaw_rad / 2.0)

    assert req.entity.name == "a200_0000/robot"
    assert req.entity.type == Entity.MODEL
    assert req.pose.position.x == 10.0
    assert abs(req.pose.orientation.z - 0.70710678) < 1e-4
    node.destroy_node()


def test_publish_goal_marker(ros_context):
    node = rclpy.create_node('test_marker_publisher')
    received_markers = []

    def marker_cb(msg):
        received_markers.append(msg)

    sub = node.create_subscription(Marker, '/a200_0000/goal_marker', marker_cb, 10)

    publish_goal_marker(node, 12.0, 4.0, frame='map', namespace='a200_0000')

    # Spin briefly to receive the marker message
    rclpy.spin_once(node, timeout_sec=0.2)

    assert len(received_markers) == 1
    m = received_markers[0]
    assert m.header.frame_id == 'map'
    assert m.pose.position.x == 12.0
    assert m.pose.position.y == 4.0
    assert m.type == Marker.SPHERE
    assert m.color.g == 1.0

    node.destroy_subscription(sub)
    node.destroy_node()
