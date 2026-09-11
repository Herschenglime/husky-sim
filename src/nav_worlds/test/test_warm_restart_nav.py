import argparse
import os
import sys
import tempfile
import threading
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor

# Ensure scripts directory is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))

from run_sweep import WarmRestartRunner  # noqa: E402


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


class MockNav2Stack(rclpy.node.Node):
    """Mock Nav2 server exposing Action, Costmap, Lifecycle, and Sim interfaces."""

    def __init__(self, namespace='test_nav', world='warehouse'):
        super().__init__('mock_nav2', namespace=namespace)
        self.cleared_local = False
        self.cleared_global = False
        self.action_goals = []

        # Costmap clearing services
        self.srv_local = self.create_service(
            ClearEntireCostmap,
            f'/{namespace}/local_costmap/clear_entirely_local_costmap',
            self._handle_clear_local
        )
        self.srv_global = self.create_service(
            ClearEntireCostmap,
            f'/{namespace}/global_costmap/clear_entirely_global_costmap',
            self._handle_clear_global
        )

        # Lifecycle GetState services
        for srv_name in ['bt_navigator', 'planner_server', 'controller_server']:
            self.create_service(
                GetState,
                f'/{namespace}/{srv_name}/get_state',
                self._handle_get_state
            )

        # NavigateToPose action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            f'/{namespace}/navigate_to_pose',
            self._execute_action_cb
        )

        # Simulation services for teleportation & markers
        try:
            from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity, DeleteEntity
            self.create_service(SetEntityPose, f'/world/{world}/set_pose', self._handle_set_pose)
            self.create_service(SpawnEntity, f'/world/{world}/create', self._handle_spawn)
            self.create_service(DeleteEntity, f'/world/{world}/remove', self._handle_remove)
        except ImportError:
            pass

    def _handle_clear_local(self, request, response):
        self.cleared_local = True
        return response

    def _handle_clear_global(self, request, response):
        self.cleared_global = True
        return response

    def _handle_get_state(self, request, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        response.current_state.label = 'active'
        return response

    def _execute_action_cb(self, goal_handle):
        self.action_goals.append(goal_handle.request.pose)
        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result

    def _handle_set_pose(self, request, response):
        response.success = True
        return response

    def _handle_spawn(self, request, response):
        response.success = True
        return response

    def _handle_remove(self, request, response):
        response.success = True
        return response


def test_warm_restart_wait_for_nav2_ready(ros_context):
    ns = 'test_nav_ready'
    mock_nav = MockNav2Stack(namespace=ns)

    executor = SingleThreadedExecutor()
    executor.add_node(mock_nav)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    with tempfile.TemporaryDirectory() as tmpdir:
        args = argparse.Namespace(
            world='warehouse',
            waypoints='',
            output_dir=tmpdir,
            sweep_dir=tmpdir,
            max_runs=1,
            gui=False,
            rviz=False,
            model_name='robot'
        )
        runner = WarmRestartRunner(args)
        runner.ns = ns

        try:
            runner.init_ros(clock_timeout=0.2, use_sim_time=False)
            ready = runner.wait_for_nav2_ready(timeout_sec=5.0)
            assert ready is True
        finally:
            if runner.node:
                runner.node.destroy_node()
            executor.shutdown()
            thread.join(timeout=1.0)
            mock_nav.destroy_node()


def test_warm_restart_costmap_clearing(ros_context):
    ns = 'test_nav_costmap'
    mock_nav = MockNav2Stack(namespace=ns)

    executor = SingleThreadedExecutor()
    executor.add_node(mock_nav)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    with tempfile.TemporaryDirectory() as tmpdir:
        args = argparse.Namespace(
            world='warehouse',
            waypoints='',
            output_dir=tmpdir,
            sweep_dir=tmpdir,
            max_runs=1,
            gui=False,
            rviz=False,
            model_name='robot'
        )
        runner = WarmRestartRunner(args)
        runner.ns = ns

        try:
            runner.init_ros(clock_timeout=0.2, use_sim_time=False)
            # Reset environment with x=1.0, y=2.0, yaw=0.0
            runner.reset_environment(1.0, 2.0, 0.0)
            assert mock_nav.cleared_local is True
            assert mock_nav.cleared_global is True
        finally:
            if runner.node:
                runner.node.destroy_node()
            executor.shutdown()
            thread.join(timeout=1.0)
            mock_nav.destroy_node()


def test_warm_restart_in_process_goal_execution(ros_context):
    ns = 'test_nav_goal'
    mock_nav = MockNav2Stack(namespace=ns)

    executor = SingleThreadedExecutor()
    executor.add_node(mock_nav)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    with tempfile.TemporaryDirectory() as tmpdir:
        args = argparse.Namespace(
            world='warehouse',
            waypoints='',
            output_dir=tmpdir,
            sweep_dir=tmpdir,
            max_runs=1,
            gui=False,
            rviz=False,
            model_name='robot',
            custom_topics=[]
        )
        runner = WarmRestartRunner(args)
        runner.ns = ns
        runner.sweep_dir = tmpdir
        runner.bag_topics = []

        try:
            runner.init_ros(clock_timeout=0.2, use_sim_time=False)
            wp = {
                'start_x': '0.0',
                'start_y': '0.0',
                'start_yaw_rad': '0.0',
                'goal_x': '3.5',
                'goal_y': '-2.0',
                'goal_yaw_rad': '1.57'
            }
            # execute_nav_goal runs in-process via BasicNavigator
            status = runner.execute_nav_goal(wp, timeout_sim=5.0)
            assert status == 'completed'
            assert len(mock_nav.action_goals) == 1
            received_pose = mock_nav.action_goals[0].pose.position
            assert abs(received_pose.x - 3.5) < 1e-4
            assert abs(received_pose.y - (-2.0)) < 1e-4
        finally:
            if runner.node:
                runner.node.destroy_node()
            executor.shutdown()
            thread.join(timeout=1.0)
            mock_nav.destroy_node()
