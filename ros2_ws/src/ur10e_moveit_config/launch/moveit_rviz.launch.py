# moveit_rviz launch.
#
# Same reason as move_group.launch.py: the generated helper ignores use_sim_time.
# RViz must also run on Isaac sim time so the interactive marker / current state
# line up with move_group.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur10e", package_name="ur10e_moveit_config"
    ).to_moveit_configs()

    rviz_config = os.path.join(
        get_package_share_directory("ur10e_moveit_config"), "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},          # <-- Isaac sim time
        ],
    )

    return LaunchDescription([rviz_node])
