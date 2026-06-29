# move_group launch.
#
# NOTE: the auto-generated version called generate_move_group_launch(moveit_config),
# which does NOT support use_sim_time. Isaac publishes /clock, so move_group MUST run
# on sim time or it can't match the (sim-stamped) joint states -> "Failed to fetch
# current robot state". So we build the node explicitly and force use_sim_time.

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur10e", package_name="ur10e_moveit_config"
    ).to_moveit_configs()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},          # <-- Isaac sim time
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription([move_group_node])
