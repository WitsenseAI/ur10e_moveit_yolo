from launch import LaunchDescription
from launch_ros.actions import Node


# The pick_place node talks to the already-running move_group via the MoveGroup
# action, so it does NOT need the MoveIt config params itself.
def generate_launch_description():
    pick = Node(
        package="object_detection_yolo", executable="pick_place",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([pick])
