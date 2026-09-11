import os
import sys
import tempfile
import threading
import time

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

# Ensure scripts directory is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))

from readiness_gate import ReadinessGate  # noqa: E402


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


class MockControllerManager(rclpy.node.Node):
    """Mock controller_manager exposing standard ros2_control services."""

    def __init__(self, namespace='test_a200', initial_controllers=None):
        super().__init__('controller_manager', namespace=namespace)
        self.controllers = dict(initial_controllers or {})
        self.load_calls = []
        self.configure_calls = []
        self.switch_calls = []
        self.load_success = True
        self.configure_success = True
        self.switch_success = True

        cm_prefix = f'/{namespace}/controller_manager' if namespace else '/controller_manager'

        self.srv_list = self.create_service(
            ListControllers,
            f'{cm_prefix}/list_controllers',
            self._handle_list
        )
        self.srv_load = self.create_service(
            LoadController,
            f'{cm_prefix}/load_controller',
            self._handle_load
        )
        self.srv_configure = self.create_service(
            ConfigureController,
            f'{cm_prefix}/configure_controller',
            self._handle_configure
        )
        self.srv_switch = self.create_service(
            SwitchController,
            f'{cm_prefix}/switch_controller',
            self._handle_switch
        )

    def _handle_list(self, request, response):
        response.controller = []
        for name, state in self.controllers.items():
            cs = ControllerState()
            cs.name = name
            cs.state = state
            response.controller.append(cs)
        return response

    def _handle_load(self, request, response):
        self.load_calls.append(request.name)
        if self.load_success:
            self.controllers[request.name] = 'unconfigured'
            response.ok = True
        else:
            response.ok = False
        return response

    def _handle_configure(self, request, response):
        self.configure_calls.append(request.name)
        if self.configure_success and request.name in self.controllers:
            self.controllers[request.name] = 'inactive'
            response.ok = True
        else:
            response.ok = False
        return response

    def _handle_switch(self, request, response):
        self.switch_calls.append(list(request.activate_controllers))
        if self.switch_success:
            for name in request.activate_controllers:
                self.controllers[name] = 'active'
            response.ok = True
            response.message = ''
        else:
            response.ok = False
            response.message = 'switch failed'
        return response


def test_gate3_controllers_already_active(ros_context):
    ns = 'test_active'
    mock_cm = MockControllerManager(
        namespace=ns,
        initial_controllers={
            'joint_state_broadcaster': 'active',
            'platform_velocity_controller': 'active'
        }
    )

    executor = SingleThreadedExecutor()
    executor.add_node(mock_cm)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        gate = ReadinessGate()
        gate.ns = ns
        gate.timeout_sec = 5.0

        success = gate._check_controllers_gate(time.monotonic())
        assert success is True
        assert len(mock_cm.load_calls) == 0
        assert len(mock_cm.configure_calls) == 0
        assert len(mock_cm.switch_calls) == 0
    finally:
        gate.destroy_node()
        executor.shutdown()
        thread.join(timeout=1.0)
        mock_cm.destroy_node()


def test_gate3_controllers_auto_recovery_from_unloaded(ros_context):
    ns = 'test_recovery_unloaded'
    mock_cm = MockControllerManager(namespace=ns, initial_controllers={})

    executor = SingleThreadedExecutor()
    executor.add_node(mock_cm)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        gate = ReadinessGate()
        gate.ns = ns
        gate.timeout_sec = 10.0

        success = gate._check_controllers_gate(time.monotonic())
        assert success is True

        assert 'joint_state_broadcaster' in mock_cm.load_calls
        assert 'platform_velocity_controller' in mock_cm.load_calls
        assert 'joint_state_broadcaster' in mock_cm.configure_calls
        assert 'platform_velocity_controller' in mock_cm.configure_calls
        assert mock_cm.controllers.get('joint_state_broadcaster') == 'active'
        assert mock_cm.controllers.get('platform_velocity_controller') == 'active'
    finally:
        gate.destroy_node()
        executor.shutdown()
        thread.join(timeout=1.0)
        mock_cm.destroy_node()


def test_gate3_controllers_partial_states(ros_context):
    """Test when one controller is unconfigured and one is already inactive."""
    ns = 'test_partial_states'
    mock_cm = MockControllerManager(
        namespace=ns,
        initial_controllers={
            'joint_state_broadcaster': 'unconfigured',
            'platform_velocity_controller': 'inactive'
        }
    )

    executor = SingleThreadedExecutor()
    executor.add_node(mock_cm)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        gate = ReadinessGate()
        gate.ns = ns
        gate.timeout_sec = 10.0

        success = gate._check_controllers_gate(time.monotonic())
        assert success is True

        # Neither should be loaded because both were already loaded
        assert len(mock_cm.load_calls) == 0
        # Only joint_state_broadcaster needed configure
        assert 'joint_state_broadcaster' in mock_cm.configure_calls
        assert 'platform_velocity_controller' not in mock_cm.configure_calls
        # Both switched to active
        assert mock_cm.controllers.get('joint_state_broadcaster') == 'active'
        assert mock_cm.controllers.get('platform_velocity_controller') == 'active'
    finally:
        gate.destroy_node()
        executor.shutdown()
        thread.join(timeout=1.0)
        mock_cm.destroy_node()


def test_call_service_sync_exception_handling(ros_context):
    """Verify _call_service_sync handles client failure without AttributeError."""
    gate = ReadinessGate()
    client = gate._get_cm_client('dummy_cm', ListControllers, 'list_controllers')

    # Calling an unavailable service with 0.1s timeout should return None cleanly
    res = gate._call_service_sync(client, ListControllers.Request(), timeout_sec=0.1)
    assert res is None
    gate.destroy_node()


def test_ensure_controllers_service_failures(ros_context):
    """Verify _ensure_controllers gracefully logs and continues when services fail."""
    ns = 'test_failures'
    mock_cm = MockControllerManager(namespace=ns, initial_controllers={})
    mock_cm.load_success = False  # Load will fail

    executor = SingleThreadedExecutor()
    executor.add_node(mock_cm)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        gate = ReadinessGate()
        gate.ns = ns
        # Should attempt to load, fail, and not raise unhandled exceptions
        gate._ensure_controllers(
            f'/{ns}/controller_manager',
            {'joint_state_broadcaster'},
            {}
        )
        assert 'joint_state_broadcaster' in mock_cm.load_calls
        assert len(mock_cm.configure_calls) == 0
        assert len(mock_cm.switch_calls) == 0
    finally:
        gate.destroy_node()
        executor.shutdown()
        thread.join(timeout=1.0)
        mock_cm.destroy_node()


def test_gate3_timeout_failure(ros_context):
    gate = ReadinessGate()
    gate.ns = 'non_existent_ns'
    gate.timeout_sec = 0.5

    start_time = time.monotonic()
    success = gate._check_controllers_gate(start_time)
    assert success is False
    gate.destroy_node()
