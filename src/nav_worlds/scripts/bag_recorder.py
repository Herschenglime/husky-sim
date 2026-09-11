#!/usr/bin/env python3
"""Programmatic Rosbag2 Recording Helper using rosbag2_py with MCAP format."""

import argparse
import os
import shutil
import sys
import time

try:
    import rclpy
    import rosbag2_py
except ImportError:
    rclpy = None
    rosbag2_py = None


class RosbagRecorder:
    """Programmatic in-process rosbag2 recorder using rosbag2_py with MCAP format."""

    def __init__(
        self,
        output_uri: str,
        topics: list[str],
        storage_id: str = 'mcap',
        use_sim_time: bool = True,
        log_level: str = 'warn',
        node_name: str = 'rosbag2_recorder',
    ):
        self.output_uri = os.path.abspath(output_uri)
        self.topics = [t for t in topics if t]
        self.storage_id = storage_id
        self.use_sim_time = use_sim_time
        self.log_level = log_level
        self.node_name = node_name
        self._recorder = None
        self._is_recording = False

    def start(self):
        """Initialize and start background recorder."""
        if rosbag2_py is None or rclpy is None:
            raise RuntimeError(
                'rosbag2_py or rclpy not found. Did you source setup.bash?'
            )

        if not rclpy.ok():
            rclpy.init()

        if self._is_recording:
            return

        # Prepare output directory
        parent_dir = os.path.dirname(self.output_uri)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)
        if os.path.exists(self.output_uri):
            shutil.rmtree(self.output_uri, ignore_errors=True)

        storage_options = rosbag2_py.StorageOptions(
            uri=self.output_uri,
            storage_id=self.storage_id,
        )

        record_options = rosbag2_py.RecordOptions()
        record_options.topics = list(self.topics)
        record_options.use_sim_time = self.use_sim_time
        record_options.disable_keyboard_controls = True

        self._recorder = rosbag2_py.Recorder(
            storage_options,
            record_options,
            log_level=self.log_level,
            node_name=self.node_name,
        )

        self._recorder.start_spin()
        self._recorder.record()
        self._is_recording = True

        if not os.path.exists(self.output_uri):
            self.stop()
            raise RuntimeError(
                f'Failed to initialize rosbag recording at {self.output_uri}'
            )

    def stop(self):
        """Stop recording and flush metadata/file buffers."""
        if self._recorder is not None:
            try:
                self._recorder.stop()
            except Exception as e:
                print(f'[WARN] Error stopping rosbag recorder: {e}', file=sys.stderr)
            try:
                self._recorder.stop_spin()
            except Exception as e:
                print(f'[WARN] Error stopping recorder executor: {e}', file=sys.stderr)
            self._recorder = None
        self._is_recording = False

    @property
    def is_recording(self) -> bool:
        """Return whether the recorder is actively running."""
        return self._is_recording

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


def main():
    parser = argparse.ArgumentParser(description='Record ROS 2 topics to MCAP bag')
    parser.add_argument('-o', '--output', required=True, help='Output bag directory')
    parser.add_argument('topics', nargs='+', help='Topics to record')
    parser.add_argument('--storage-id', default='mcap', choices=['mcap', 'sqlite3'])
    parser.add_argument('--use-sim-time', action='store_true', default=True)
    parser.add_argument('--no-sim-time', dest='use_sim_time', action='store_false')
    args = parser.parse_args()

    recorder = RosbagRecorder(
        args.output,
        args.topics,
        storage_id=args.storage_id,
        use_sim_time=args.use_sim_time,
    )
    recorder.start()
    print(f'Recording {len(args.topics)} topics to {args.output} ({args.storage_id})...')
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\nStopping recording...')
    finally:
        recorder.stop()
        print('Finished.')


if __name__ == '__main__':
    main()
