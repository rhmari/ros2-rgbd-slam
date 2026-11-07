#!/usr/bin/env python3
"""ROS 2 wrapper around vo_core.RGBDVisualOdometry.

Subscribes  /camera/rgb/image_raw (bgr8), /camera/depth/image_raw (32FC1, m), /camera/rgb/camera_info
Publishes   /my_vo/odom (nav_msgs/Odometry, odom -> camera_link) and the matching tf,
            so it can replace RTAB-Map's odometry and feed RTAB-Map's SLAM back end.
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vo_core import RGBDVisualOdometry, VOParams  # noqa: E402

# camera_link (x forward, y left, z up) <- optical frame (x right, y down, z forward)
T_LINK_OPT = np.eye(4)
T_LINK_OPT[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
T_OPT_LINK = np.linalg.inv(T_LINK_OPT)


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class RGBDVONode(Node):
    def __init__(self):
        super().__init__("my_rgbd_vo")
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "camera_link").value
        params = VOParams(
            n_features=int(self.declare_parameter("n_features", 2000).value),
            keyframe_inliers=int(self.declare_parameter("keyframe_inliers", 150).value),
        )
        self.vo = RGBDVisualOdometry(params)

        self.pub = self.create_publisher(Odometry, "/my_vo/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.buf = {}  # stamp -> {"rgb": msg, "depth": msg, "info": msg}
        self.create_subscription(Image, "/camera/rgb/image_raw", lambda m: self.add("rgb", m), 10)
        self.create_subscription(Image, "/camera/depth/image_raw", lambda m: self.add("depth", m), 10)
        self.create_subscription(CameraInfo, "/camera/rgb/camera_info", lambda m: self.add("info", m), 10)

        self.n = self.n_lost = self.n_kf = 0
        self.t_total = 0.0
        self.get_logger().info("My RGB-D VO ready (ORB + frame-to-keyframe PnP/RANSAC)")

    # Exact-time synchronisation: process a stamp once RGB, depth and camera info all arrived.
    def add(self, kind, msg):
        key = stamp_ns(msg)
        entry = self.buf.setdefault(key, {})
        entry[kind] = msg
        if len(entry) == 3:
            del self.buf[key]
            for old in [k for k in self.buf if k < key]:  # drop incomplete older sets
                del self.buf[old]
            self.process(entry["rgb"], entry["depth"], entry["info"])

    def process(self, rgb, depth, info):
        t0 = time.perf_counter()
        bgr = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, 3)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        depth_m = np.frombuffer(depth.data, np.float32).reshape(depth.height, depth.width)
        K = np.array(info.k, dtype=np.float64).reshape(3, 3)

        T_w_opt, ok = self.vo.process(gray, depth_m, K)
        # Express the optical-frame trajectory as camera_link poses in an odom frame that is
        # aligned with the first camera_link pose.
        T_odom_link = T_LINK_OPT @ T_w_opt @ T_OPT_LINK
        self.publish(rgb.header.stamp, T_odom_link, ok)

        self.t_total += time.perf_counter() - t0
        self.n += 1
        self.n_lost += not ok
        self.n_kf += self.vo.last.get("new_keyframe", False)
        if self.n % 100 == 0:
            self.get_logger().info(
                f"{self.n} frames | inliers {self.vo.last.get('inliers', 0)}"
                f"/{self.vo.last.get('matches', 0)} | keyframes {self.n_kf} | lost {self.n_lost} | "
                f"{1000 * self.t_total / self.n:.1f} ms/frame")

    def publish(self, stamp, T, ok):
        q = Rotation.from_matrix(T[:3, :3]).as_quat()  # x, y, z, w
        t = T[:3, 3]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        p = odom.pose.pose
        p.position.x, p.position.y, p.position.z = map(float, t)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        var_t, var_r = (1e-4, 1e-3) if ok else (9999.0, 9999.0)
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        for i, v in enumerate([var_t] * 3 + [var_r] * 3):
            odom.pose.covariance[i * 7] = v
            odom.twist.covariance[i * 7] = v
        self.pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, t)
        tf.transform.rotation = p.orientation
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = RGBDVONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"done: {node.n} frames, {node.n_lost} lost, {node.n_kf} keyframes, "
            f"{1000 * node.t_total / max(node.n, 1):.1f} ms/frame")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
