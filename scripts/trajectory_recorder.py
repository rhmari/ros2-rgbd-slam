#!/usr/bin/env python3
"""Record the estimated camera trajectory in TUM format for evaluation with evo.

Writes two files (one line per odometry message: "t tx ty tz qx qy qz qw"):
  vo_traj.txt    pure visual odometry  (odom -> camera_link)
  slam_traj.txt  online SLAM estimate  (map -> camera_link = map->odom * odom->camera_link),
                 i.e. odometry corrected by RTAB-Map's loop closures / graph optimization
"""
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def qrot(q, v):
    """Rotate vector v by unit quaternion q = (x, y, z, w)."""
    qv = np.array([v[0], v[1], v[2], 0.0])
    q_conj = np.array([-q[0], -q[1], -q[2], q[3]])
    return qmul(qmul(q, qv), q_conj)[:3]


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")
        odom_topic = self.declare_parameter("odom_topic", "/rtabmap/odom").value
        out_dir = self.declare_parameter("out_dir", "/results").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        # Empty = take the odometry frame from the odometry messages themselves.
        self.odom_frame = self.declare_parameter("odom_frame", "").value

        os.makedirs(out_dir, exist_ok=True)
        self.vo_file = open(os.path.join(out_dir, "vo_traj.txt"), "w")
        self.slam_file = open(os.path.join(out_dir, "slam_traj.txt"), "w")
        self.n = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        self.get_logger().info(f"Recording {odom_topic} -> {out_dir}")

    @staticmethod
    def line(t, p, q):
        return f"{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n"

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos, ori = msg.pose.pose.position, msg.pose.pose.orientation
        p_oc = np.array([pos.x, pos.y, pos.z])
        q_oc = np.array([ori.x, ori.y, ori.z, ori.w])
        if np.linalg.norm(q_oc) < 1e-6:
            return  # RTAB-Map publishes a null pose when odometry is lost

        self.vo_file.write(self.line(t, p_oc, q_oc))
        self.vo_file.flush()

        odom_frame = self.odom_frame or msg.header.frame_id.lstrip("/")
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, odom_frame, rclpy.time.Time())
        except TransformException:
            return  # map->odom not published yet
        tr, rot = tf.transform.translation, tf.transform.rotation
        p_mo = np.array([tr.x, tr.y, tr.z])
        q_mo = np.array([rot.x, rot.y, rot.z, rot.w])

        p_mc = p_mo + qrot(q_mo, p_oc)
        q_mc = qmul(q_mo, q_oc)
        q_mc /= np.linalg.norm(q_mc)
        self.slam_file.write(self.line(t, p_mc, q_mc))
        self.slam_file.flush()

        self.n += 1
        if self.n % 200 == 0:
            self.get_logger().info(f"{self.n} poses recorded")

    def destroy_node(self):
        self.vo_file.close()
        self.slam_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
