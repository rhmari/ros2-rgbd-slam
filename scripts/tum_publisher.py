#!/usr/bin/env python3
"""Replay a TUM RGB-D sequence folder as ROS 2 topics.

Reads rgb.txt / depth.txt from an extracted TUM sequence, pairs each RGB frame
with the closest depth frame, and publishes:
  /camera/rgb/image_raw     sensor_msgs/Image   (bgr8)
  /camera/depth/image_raw   sensor_msgs/Image   (32FC1, metres, registered to RGB)
  /camera/rgb/camera_info   sensor_msgs/CameraInfo
  /clock                    rosgraph_msgs/Clock (dataset time, so stamps match groundtruth.txt)

Why not the official .bag? It is ROS 1, its frame ids start with "/" (rejected by tf2 in ROS 2)
and it ships the mocap ground truth on /tf, which clashes with the SLAM tf tree.
"""
import array
import os
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

# TUM recommends the ROS default intrinsics for the registered depth/RGB pair.
FX, FY, CX, CY = 525.0, 525.0, 319.5, 239.5
DEPTH_SCALE = 5000.0  # TUM depth PNG: 5000 units = 1 m


def read_list(path):
    entries = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            stamp, fname = line.split()[:2]
            entries.append((float(stamp), fname))
    return entries


def associate(rgb, depth, max_dt=0.02):
    """Pair each RGB frame with the nearest unused depth frame (like TUM's associate.py)."""
    depth_t = np.array([t for t, _ in depth])
    used, pairs = set(), []
    for t, rgb_file in rgb:
        i = int(np.argmin(np.abs(depth_t - t)))
        if abs(depth_t[i] - t) < max_dt and i not in used:
            used.add(i)
            pairs.append((t, rgb_file, depth[i][1]))
    return pairs


def to_stamp(t):
    ns = int(round(t * 1e9))
    msg = TimeMsg()
    msg.sec, msg.nanosec = ns // 1_000_000_000, ns % 1_000_000_000
    return msg


def make_image(arr, encoding, stamp, frame_id):
    arr = np.ascontiguousarray(arr)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = arr.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = arr.strides[0]
    # array.array is rclpy's fast path; assigning raw bytes makes rclpy check every byte
    # in Python (~2 MB per frame), which capped playback at ~3 fps.
    msg.data = array.array("B", arr.tobytes())
    return msg


def make_info(stamp, frame_id, width, height):
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width, msg.height = width, height
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0] * 5
    msg.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


class TumPublisher(Node):
    def __init__(self):
        super().__init__("tum_publisher")
        self.dataset = self.declare_parameter("dataset", "/data/rgbd_dataset_freiburg1_xyz").value
        self.speed = float(self.declare_parameter("speed", 1.0).value)
        self.startup_delay = float(self.declare_parameter("startup_delay", 8.0).value)
        self.frame_id = self.declare_parameter("frame_id", "camera_optical_frame").value

        self.pub_rgb = self.create_publisher(Image, "/camera/rgb/image_raw", 10)
        self.pub_depth = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/rgb/camera_info", 10)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)

        rgb = read_list(os.path.join(self.dataset, "rgb.txt"))
        depth = read_list(os.path.join(self.dataset, "depth.txt"))
        self.pairs = associate(rgb, depth)
        self.get_logger().info(
            f"{len(self.pairs)} RGB-D pairs from {self.dataset} (speed x{self.speed})")

    def publish_clock(self, t):
        msg = Clock()
        msg.clock = to_stamp(t)
        self.pub_clock.publish(msg)

    def run(self):
        t0 = self.pairs[0][0]
        # Publish the start time while SLAM nodes come up, so sim-time nodes are not stuck at 0.
        end_wait = time.monotonic() + self.startup_delay
        while time.monotonic() < end_wait and rclpy.ok():
            self.publish_clock(t0)
            time.sleep(0.1)

        start = time.monotonic()
        for k, (t, rgb_file, depth_file) in enumerate(self.pairs):
            if not rclpy.ok():
                break
            target = start + (t - t0) / self.speed
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)

            bgr = cv2.imread(os.path.join(self.dataset, rgb_file), cv2.IMREAD_COLOR)
            depth_raw = cv2.imread(os.path.join(self.dataset, depth_file), cv2.IMREAD_UNCHANGED)
            depth_m = depth_raw.astype(np.float32) / DEPTH_SCALE

            stamp = to_stamp(t)
            self.publish_clock(t)
            self.pub_rgb.publish(make_image(bgr, "bgr8", stamp, self.frame_id))
            self.pub_depth.publish(make_image(depth_m, "32FC1", stamp, self.frame_id))
            self.pub_info.publish(make_info(stamp, self.frame_id, bgr.shape[1], bgr.shape[0]))

            if k % 100 == 0:
                self.get_logger().info(f"frame {k}/{len(self.pairs)}")

        self.get_logger().info("Sequence finished.")


def main():
    rclpy.init()
    node = TumPublisher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
