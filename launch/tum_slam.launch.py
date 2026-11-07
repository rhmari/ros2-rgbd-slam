"""TUM RGB-D sequence -> RTAB-Map (RGB-D visual odometry + SLAM) -> trajectory files + Foxglove.

Usage (inside the container):
  ros2 launch /ws/launch/tum_slam.launch.py dataset:=/data/rgbd_dataset_freiburg1_xyz speed:=1.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dataset = LaunchConfiguration("dataset")
    speed = LaunchConfiguration("speed")
    odom_topic = LaunchConfiguration("odom_topic")

    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"), "launch", "rtabmap.launch.py")

    # Fixed mount between the camera body frame (x forward) and the optical frame (z forward).
    camera_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "-1.5707963", "--pitch", "0", "--yaw", "-1.5707963",
                   "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame"],
        parameters=[{"use_sim_time": True}],
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            "rgb_topic": "/camera/rgb/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/rgb/camera_info",
            "frame_id": "camera_link",
            "vo_frame_id": "odom",           # avoid the namespaced "/rtabmap/odom" frame
            "use_sim_time": "true",
            "approx_sync": "false",          # RGB and depth share the same stamp
            "database_path": "/results/rtabmap.db",
            "rtabmap_args": "--delete_db_on_start",
            "rviz": "false",
            "rtabmap_viz": "false",
        }.items(),
    )

    recorder = ExecuteProcess(
        cmd=["python3", "/ws/scripts/trajectory_recorder.py", "--ros-args",
             "-p", "use_sim_time:=true", "-p", ["odom_topic:=", odom_topic]],
        output="screen",
    )

    publisher = ExecuteProcess(
        cmd=["python3", "/ws/scripts/tum_publisher.py", "--ros-args",
             "-p", ["dataset:=", dataset], "-p", ["speed:=", speed]],
        output="screen",
    )

    foxglove = Node(
        package="foxglove_bridge", executable="foxglove_bridge",
        parameters=[{"port": 8765, "use_sim_time": True}],
    )

    # When the sequence is over, give SLAM a few seconds to finish, then stop everything.
    stop_when_done = RegisterEventHandler(OnProcessExit(
        target_action=publisher,
        on_exit=[TimerAction(period=5.0, actions=[Shutdown(reason="sequence finished")])],
    ))

    return LaunchDescription([
        DeclareLaunchArgument("dataset", default_value="/data/rgbd_dataset_freiburg1_xyz"),
        DeclareLaunchArgument("speed", default_value="1.0",
                              description="Playback speed factor (lower it if odometry gets lost)"),
        DeclareLaunchArgument("odom_topic", default_value="/rtabmap/odom"),
        camera_tf, rtabmap, foxglove, recorder, publisher, stop_when_done,
    ])
