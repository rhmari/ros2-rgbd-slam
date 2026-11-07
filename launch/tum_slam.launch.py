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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    dataset = LaunchConfiguration("dataset")
    speed = LaunchConfiguration("speed")
    run = LaunchConfiguration("run")
    intrinsics = LaunchConfiguration("intrinsics")
    detection_rate = LaunchConfiguration("detection_rate")
    odometry = LaunchConfiguration("odometry")
    use_mine = PythonExpression(["'", odometry, "' == 'mine'"])
    # With odometry:=mine, RTAB-Map's own odometry is switched off and its SLAM back end
    # consumes /my_vo/odom instead.
    rtabmap_vo = PythonExpression(["'false' if '", odometry, "' == 'mine' else 'true'"])
    odom_topic = PythonExpression(["'/my_vo/odom' if '", odometry, "' == 'mine' else '/rtabmap/odom'"])

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
            "visual_odometry": rtabmap_vo,
            "odom_topic": odom_topic,
            "approx_sync": "false",          # RGB and depth share the same stamp
            "database_path": ["/results/", run, "/rtabmap.db"],
            # DetectionRate: how often (Hz) a new node is added to the map graph; 0 = every frame.
            "rtabmap_args": ["--delete_db_on_start --Rtabmap/DetectionRate ", detection_rate],
            "rviz": "false",
            "rtabmap_viz": "false",
        }.items(),
    )

    recorder = ExecuteProcess(
        cmd=["python3", "/ws/scripts/trajectory_recorder.py", "--ros-args",
             "-p", "use_sim_time:=true", "-p", ["odom_topic:=", odom_topic],
             "-p", ["out_dir:=/results/", run]],
        output="screen",
    )

    my_vo = ExecuteProcess(
        cmd=["python3", "/ws/scripts/rgbd_vo_node.py", "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
        condition=IfCondition(use_mine),
    )

    publisher = ExecuteProcess(
        cmd=["python3", "/ws/scripts/tum_publisher.py", "--ros-args",
             "-p", ["dataset:=", dataset], "-p", ["speed:=", speed],
             "-p", ["intrinsics:=", intrinsics]],
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
        DeclareLaunchArgument("odometry", default_value="rtabmap",
                              description="'rtabmap' (RTAB-Map's RGB-D odometry) or 'mine' (rgbd_vo_node)"),
        DeclareLaunchArgument("run", default_value="default",
                              description="Subfolder of /results for this run's outputs"),
        DeclareLaunchArgument("intrinsics", default_value="fr1",
                              description="'default' (ROS 525/525) or 'fr1' (calibrated + undistortion)"),
        DeclareLaunchArgument("detection_rate", default_value="0",
                              description="RTAB-Map map update rate in Hz (0 = every frame)"),
        camera_tf, rtabmap, my_vo, foxglove, recorder, publisher, stop_when_done,
    ])
