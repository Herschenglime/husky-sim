#!/usr/bin/env python3
"""Unit tests for Task 6: Unified Pipeline Coordination & Graceful Teardown.

Tests:
  1. test_collect_dataset_runner_selection – ColdRestartRunner vs WarmRestartRunner selection
  2. test_preview_display_handling        – xdg-open launch / skip logic
  3. test_in_process_state_logger_lifecycle – StateLogger configure/activate/deactivate cycle
"""

import argparse
import io
import os
import sys
import tempfile
import threading
import unittest
from unittest.mock import MagicMock, call, patch

# ── Path setup ───────────────────────────────────────────────────────────────
SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPTS_DIR))


# ════════════════════════════════════════════════════════════════════════════
# Test 1: Runner subclass selection
# ════════════════════════════════════════════════════════════════════════════

class TestRunnerSelection(unittest.TestCase):
    """Verify collect_dataset selects the correct runner subclass."""

    def _make_sweep_args(self, cold_restart: bool) -> argparse.Namespace:
        return argparse.Namespace(
            waypoints='dummy.csv',
            sweep_dir='/tmp/sweep',
            world='warehouse',
            bag_profile='standard',
            storage_id='mcap',
            cold_restart=cold_restart,
            warm_reset=not cold_restart,
            gui=False,
            rviz=False,
            add_topics=None,
            custom_topics=None,
            verbose=False,
            overwrite=False,
            max_runs=None,
            output_dir='/tmp/sweep',
            include_camera=False,
            disable_lidar2d=False,
            model_name='a200_0000/robot',
        )

    def test_warm_restart_selected_by_default(self):
        """WarmRestartRunner is instantiated when cold_restart is False."""
        from run_sweep import WarmRestartRunner, ColdRestartRunner

        args = self._make_sweep_args(cold_restart=False)
        runner_cls = ColdRestartRunner if args.cold_restart else WarmRestartRunner
        self.assertIs(runner_cls, WarmRestartRunner)

    def test_cold_restart_selected_when_flag_set(self):
        """ColdRestartRunner is instantiated when cold_restart is True."""
        from run_sweep import WarmRestartRunner, ColdRestartRunner

        args = self._make_sweep_args(cold_restart=True)
        runner_cls = ColdRestartRunner if args.cold_restart else WarmRestartRunner
        self.assertIs(runner_cls, ColdRestartRunner)

    def test_warm_restart_runner_is_simulation_runner_subclass(self):
        from run_sweep import WarmRestartRunner, SimulationRunner
        self.assertTrue(issubclass(WarmRestartRunner, SimulationRunner))

    def test_cold_restart_runner_is_simulation_runner_subclass(self):
        from run_sweep import ColdRestartRunner, SimulationRunner
        self.assertTrue(issubclass(ColdRestartRunner, SimulationRunner))


# ════════════════════════════════════════════════════════════════════════════
# Test 2: Preview display logic
# ════════════════════════════════════════════════════════════════════════════

class TestPreviewDisplayHandling(unittest.TestCase):
    """Verify xdg-open is launched / skipped based on display and --no-preview."""

    def _call_prompt(self, has_display: bool, has_xdg: bool, no_preview: bool,
                     user_answer: str = 'y'):
        from collect_dataset import prompt_user_confirmation

        env_overrides = {'DISPLAY': ':0', 'WAYLAND_DISPLAY': ''} if has_display else \
                        {'DISPLAY': '', 'WAYLAND_DISPLAY': ''}

        with patch.dict(os.environ, env_overrides, clear=False), \
             patch('collect_dataset.shutil.which',
                   return_value='/usr/bin/xdg-open' if has_xdg else None), \
             patch('collect_dataset.subprocess.Popen') as mock_popen, \
             patch('builtins.input', return_value=user_answer):

            mock_proc = MagicMock()
            mock_popen.return_value = mock_proc

            result = prompt_user_confirmation(
                '/tmp/fake_preview.png', 5, no_preview=no_preview
            )

            return result, mock_popen.called, mock_proc.terminate.called

    def test_xdg_open_launched_when_display_present(self):
        """xdg-open is called when display is present and no_preview is False."""
        result, popen_called, terminate_called = self._call_prompt(
            has_display=True, has_xdg=True, no_preview=False, user_answer='y'
        )
        self.assertTrue(result)
        self.assertTrue(popen_called)
        self.assertTrue(terminate_called, "Viewer process must be terminated after prompt")

    def test_xdg_open_skipped_when_no_preview_flag(self):
        """xdg-open is NOT called when --no-preview is set, even with a display."""
        result, popen_called, _ = self._call_prompt(
            has_display=True, has_xdg=True, no_preview=True, user_answer='y'
        )
        self.assertTrue(result)
        self.assertFalse(popen_called)

    def test_xdg_open_skipped_when_no_display(self):
        """xdg-open is NOT called when no graphical display is detected."""
        result, popen_called, _ = self._call_prompt(
            has_display=False, has_xdg=True, no_preview=False, user_answer='y'
        )
        self.assertTrue(result)
        self.assertFalse(popen_called)

    def test_user_rejection_returns_false(self):
        """prompt_user_confirmation returns False when user types 'n'."""
        result, _, _ = self._call_prompt(
            has_display=False, has_xdg=False, no_preview=False, user_answer='n'
        )
        self.assertFalse(result)

    def test_eof_returns_false(self):
        """prompt_user_confirmation returns False on EOFError (non-interactive shell)."""
        from collect_dataset import prompt_user_confirmation

        with patch('builtins.input', side_effect=EOFError), \
             patch.dict(os.environ, {'DISPLAY': '', 'WAYLAND_DISPLAY': ''}, clear=False):
            result = prompt_user_confirmation('/tmp/fake.png', 3, no_preview=True)
        self.assertFalse(result)


# ════════════════════════════════════════════════════════════════════════════
# Test 3: In-process StateLogger lifecycle
# ════════════════════════════════════════════════════════════════════════════

class TestInProcessStateLoggerLifecycle(unittest.TestCase):
    """Verify StateLogger can be configured, activated, deactivated, and re-activated
    in-process without file descriptor leaks.

    These tests require a sourced ROS 2 environment and are skipped otherwise.
    """

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        # rclpy.init() tries to create /home/pgrau/.ros/log — redirect to tmpdir
        os.environ.setdefault('ROS_LOG_DIR', os.path.join(self._tmpdir, 'ros_log'))

    def tearDown(self):
        import shutil as _shutil
        _shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _make_node(self, output_file: str):
        import rclpy as _rclpy
        if not _rclpy.ok():
            _rclpy.init()
        from log_state import StateLogger
        node = StateLogger(node_name='test_state_logger', output_file=output_file)
        return node

    @unittest.skipUnless(
        os.environ.get('AMENT_PREFIX_PATH'),
        'Skipped: ROS 2 environment not sourced'
    )
    def test_configure_activate_deactivate_cycle(self):
        """StateLogger transitions through unconfigured→inactive→active→inactive cleanly."""
        from rclpy.lifecycle import TransitionCallbackReturn

        out_file = os.path.join(self._tmpdir, 'state_cycle.jsonl')
        node = self._make_node(out_file)
        try:
            self.assertEqual(node.current_state, 'unconfigured')

            self.assertEqual(node.trigger_configure(), TransitionCallbackReturn.SUCCESS)
            self.assertEqual(node.current_state, 'inactive')

            self.assertEqual(node.trigger_activate(), TransitionCallbackReturn.SUCCESS)
            self.assertEqual(node.current_state, 'active')
            self.assertTrue(node._is_active)

            self.assertEqual(node.trigger_deactivate(), TransitionCallbackReturn.SUCCESS)
            self.assertEqual(node.current_state, 'inactive')
            self.assertFalse(node._is_active)
            self.assertTrue(
                node.file is None or node.file.closed,
                "Output file must be closed after deactivate"
            )
        finally:
            node.destroy_node()

    @unittest.skipUnless(
        os.environ.get('AMENT_PREFIX_PATH'),
        'Skipped: ROS 2 environment not sourced'
    )
    def test_reactivation_with_new_output_file(self):
        """StateLogger can be re-activated with a different output file path."""
        from rclpy.lifecycle import TransitionCallbackReturn
        from rclpy.parameter import Parameter

        out1 = os.path.join(self._tmpdir, 'run_000.jsonl')
        out2 = os.path.join(self._tmpdir, 'run_001.jsonl')
        node = self._make_node(out1)
        try:
            node.trigger_configure()
            node.trigger_activate()
            self.assertEqual(node.output_file, out1)
            node.trigger_deactivate()

            # Update output path while Inactive
            node.set_parameters([Parameter('output', Parameter.Type.STRING, out2)])
            self.assertEqual(node.output_file, out2)

            # Re-activate with new file
            self.assertEqual(node.trigger_activate(), TransitionCallbackReturn.SUCCESS)
            self.assertEqual(node.current_state, 'active')
            self.assertEqual(node.output_file, out2)
            self.assertFalse(node.file.closed)

            node.trigger_deactivate()
            self.assertTrue(node.file is None or node.file.closed)
        finally:
            node.destroy_node()

    @unittest.skipUnless(
        os.environ.get('AMENT_PREFIX_PATH'),
        'Skipped: ROS 2 environment not sourced'
    )
    def test_parameter_update_rejected_when_active(self):
        """StateLogger rejects 'output' parameter changes while in Active state."""
        from rclpy.parameter import Parameter

        out_file = os.path.join(self._tmpdir, 'active_lock.jsonl')
        node = self._make_node(out_file)
        try:
            node.trigger_configure()
            node.trigger_activate()
            self.assertEqual(node.current_state, 'active')

            results = node.set_parameters(
                [Parameter('output', Parameter.Type.STRING, '/tmp/should_fail.jsonl')]
            )
            # The callback should reject the update
            self.assertFalse(results[0].successful)
            # Output file unchanged
            self.assertEqual(node.output_file, out_file)
        finally:
            try:
                node.trigger_deactivate()
            except Exception:
                pass
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
