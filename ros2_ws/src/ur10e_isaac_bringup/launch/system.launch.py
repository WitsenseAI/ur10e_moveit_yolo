
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(pkg, launch_file, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", launch_file])),
        condition=condition,
    )


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    detection = LaunchConfiguration("detection")
    add_table = LaunchConfiguration("add_table")
    pick = LaunchConfiguration("pick")

    bringup = _include("ur10e_isaac_bringup", "bringup.launch.py")
    move_group = _include("ur10e_moveit_config", "move_group.launch.py")
    rviz_node = _include("ur10e_moveit_config", "moveit_rviz.launch.py",
                         condition=IfCondition(rviz))
    detect = _include("object_detection_yolo", "detection.launch.py",
                      condition=IfCondition(detection))
    pick_place = _include("object_detection_yolo", "pick_place.launch.py",
                          condition=IfCondition(pick))

    # add_table.py talks to move_group's /apply_planning_scene; give move_group a few
    # seconds to come up first (the script also waits for the service on its own).
    table_proc = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=["python3", os.path.expanduser(
                "~/witsense/moveit-ros/ros2_ws/src/add_table.py")],
            output="screen")],
        condition=IfCondition(add_table),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("detection", default_value="true"),
        DeclareLaunchArgument("add_table", default_value="true"),
        DeclareLaunchArgument("pick", default_value="false",
                              description="also run pick&place (moves the robot)"),
        bringup,
        move_group,
        rviz_node,
        detect,
        table_proc,
        pick_place,
    ])
