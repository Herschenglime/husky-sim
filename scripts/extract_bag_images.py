#!/usr/bin/env python3
"""Extract images and common TurtleBot sensor data from ROS 2 bags."""

import argparse
import csv
import json
import sys
from contextlib import ExitStack
from pathlib import Path

import cv2
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_INPUT = Path("/home/robopi/tb4_bags")
DEFAULT_OUTPUT = Path("/home/robopi/ros2_ws/bag_exports")
DEFAULT_TOPIC = "/oakd/rgb/preview/image_raw"
DATA_TOPICS = ("/cmd_vel", "/odom", "/scan", "/tf", "/tf_static")


def find_bags(input_path: Path) -> list[Path]:
    """Return bag directories found at or directly below input_path."""
    if (input_path / "metadata.yaml").is_file():
        return [input_path]

    if not input_path.is_dir():
        raise FileNotFoundError(f"Input directory does not exist: {input_path}")

    return sorted(
        path for path in input_path.iterdir()
        if path.is_dir() and (path / "metadata.yaml").is_file()
    )


def extract_bag(
    bag_path: Path,
    output_root: Path,
    topic: str,
    extension: str,
    jpeg_quality: int,
    max_frames: int | None,
) -> dict[str, int]:
    """Extract camera and sensor messages from one bag."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if topic not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise ValueError(
            f"Topic {topic!r} is not in {bag_path.name}. Available: {available}"
        )
    if topic_types[topic] != "sensor_msgs/msg/Image":
        raise ValueError(
            f"Topic {topic!r} has type {topic_types[topic]}, not sensor_msgs/msg/Image"
        )

    message_types = {
        name: get_message(type_name)
        for name, type_name in topic_types.items()
        if name == topic or name in DATA_TOPICS
    }
    bridge = CvBridge()
    mission_output = output_root / bag_path.name
    image_output = mission_output / "images"
    image_output.mkdir(parents=True, exist_ok=True)

    counts = {name: 0 for name in message_types}
    with ExitStack() as stack:
        def make_writer(filename: str, header: list[str]) -> csv.writer:
            csv_file = stack.enter_context(
                (mission_output / filename).open("w", newline="", encoding="utf-8")
            )
            result = csv.writer(csv_file)
            result.writerow(header)
            return result

        image_writer = make_writer(
            "timestamps.csv",
            ["frame", "filename", "bag_timestamp_ns", "header_timestamp_ns"]
        )
        cmd_writer = make_writer(
            "cmd_vel.csv",
            ["bag_timestamp_ns", "linear_x", "linear_y", "linear_z",
             "angular_x", "angular_y", "angular_z"],
        )
        odom_writer = make_writer(
            "odom.csv",
            ["bag_timestamp_ns", "header_timestamp_ns", "frame_id", "child_frame_id",
             "position_x", "position_y", "position_z", "orientation_x",
             "orientation_y", "orientation_z", "orientation_w", "linear_x",
             "linear_y", "linear_z", "angular_x", "angular_y", "angular_z",
             "pose_covariance", "twist_covariance"],
        )
        scan_writer = make_writer(
            "scan.csv",
            ["bag_timestamp_ns", "header_timestamp_ns", "frame_id", "angle_min",
             "angle_max", "angle_increment", "time_increment", "scan_time",
             "range_min", "range_max", "ranges", "intensities"],
        )
        tf_writer = make_writer(
            "tf.csv",
            ["bag_timestamp_ns", "header_timestamp_ns", "topic", "parent_frame",
             "child_frame", "translation_x", "translation_y", "translation_z",
             "rotation_x", "rotation_y", "rotation_z", "rotation_w"],
        )

        while reader.has_next():
            read_topic, data, bag_timestamp_ns = reader.read_next()
            if read_topic not in message_types:
                continue

            message = deserialize_message(data, message_types[read_topic])
            counts[read_topic] += 1

            if read_topic == topic:
                frame = counts[read_topic] - 1
                image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                filename = f"frame_{frame:06d}.{extension}"
                image_path = image_output / filename
                options = []
                if extension in ("jpg", "jpeg"):
                    options = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
                if not cv2.imwrite(str(image_path), image, options):
                    raise RuntimeError(f"Could not write image: {image_path}")
                header_ns = stamp_to_ns(message.header.stamp)
                image_writer.writerow(
                    [frame, str(Path("images") / filename), bag_timestamp_ns, header_ns]
                )
                if max_frames is not None and counts[read_topic] >= max_frames:
                    break

            elif read_topic == "/cmd_vel":
                cmd_writer.writerow([
                    bag_timestamp_ns,
                    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
                    message.twist.angular.x, message.twist.angular.y, message.twist.angular.z,
                ])

            elif read_topic == "/odom":
                pose = message.pose.pose
                twist = message.twist.twist
                odom_writer.writerow([
                    bag_timestamp_ns, stamp_to_ns(message.header.stamp),
                    message.header.frame_id, message.child_frame_id,
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w,
                    twist.linear.x, twist.linear.y, twist.linear.z,
                    twist.angular.x, twist.angular.y, twist.angular.z,
                    json.dumps(list(message.pose.covariance)),
                    json.dumps(list(message.twist.covariance)),
                ])

            elif read_topic == "/scan":
                scan_writer.writerow([
                    bag_timestamp_ns, stamp_to_ns(message.header.stamp),
                    message.header.frame_id, message.angle_min, message.angle_max,
                    message.angle_increment, message.time_increment, message.scan_time,
                    message.range_min, message.range_max,
                    json.dumps(list(message.ranges)),
                    json.dumps(list(message.intensities)),
                ])

            elif read_topic in ("/tf", "/tf_static"):
                for transform in message.transforms:
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    tf_writer.writerow([
                        bag_timestamp_ns, stamp_to_ns(transform.header.stamp), read_topic,
                        transform.header.frame_id, transform.child_frame_id,
                        translation.x, translation.y, translation.z,
                        rotation.x, rotation.y, rotation.z, rotation.w,
                    ])

    summary = ", ".join(
        f"{name}={count}" for name, count in counts.items()
    )
    print(f"{bag_path.name}: {summary} -> {mission_output}")
    return counts


def stamp_to_ns(stamp) -> int:
    """Convert a ROS builtin_interfaces/Time value to nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Extract RGB images, velocity commands, odometry, laser scans, and "
            "transforms from every ROS 2 bag in an input directory."
        )
    )
    parser.add_argument(
        "input", nargs="?", type=Path, default=DEFAULT_INPUT,
        help=f"bag directory or directory containing bags (default: {DEFAULT_INPUT})",
    )
    parser.add_argument(
        "-o", "--output", type=Path, default=DEFAULT_OUTPUT,
        help=f"output directory (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--topic", default=DEFAULT_TOPIC,
        help=f"sensor_msgs/Image topic (default: {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--format", choices=("jpg", "png"), default="png",
        help="output image format (default: png)",
    )
    parser.add_argument(
        "--jpeg-quality", type=int, choices=range(1, 101), default=100,
        metavar="1-100", help="JPEG quality (default: 100)",
    )
    parser.add_argument(
        "--max-frames", type=int,
        help="extract at most this many frames per bag (useful for testing)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        bags = find_bags(args.input.expanduser().resolve())
        if not bags:
            print(f"No ROS 2 bags found in {args.input}", file=sys.stderr)
            return 1

        output = args.output.expanduser().resolve()
        totals: dict[str, int] = {}
        for bag in bags:
            counts = extract_bag(
                bag, output, args.topic, args.format,
                args.jpeg_quality, args.max_frames,
            )
            for topic, count in counts.items():
                totals[topic] = totals.get(topic, 0) + count
        summary = ", ".join(f"{name}={count}" for name, count in totals.items())
        print(f"Done: {len(bags)} bag(s), {summary} -> {output}")
        return 0
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
