
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
     use_sim_time= LaunchConfiguration("use_sim_time")

     description_pkg= get_package_share_directory("ur10e_robot_description")
     xacro_file= os.path.join(description_pkg, "urdf", "ur10e_robotiq.urdf.xacro")

     robot_description = {
          "robot_description" : ParameterValue(
               Command([FindExecutable(name = "xacro"), " ", xacro_file]),
               value_type=str,
          )
     }

     controllers_yaml= PathJoinSubstitution(
          [FindPackageShare("ur10e_moveit_config"), "config", "ros2_controllers.yaml"]
     )

     
     return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="controller_manager", executable="ros2_control_node",
            output="screen",
            parameters=[robot_description, controllers_yaml, {"use_sim_time": use_sim_time}],
        ),
        # Spawners â names MUST match ros2_controllers.yaml / moveit_controllers.yaml
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_controller", "-c", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["gripper_controller", "-c", "/controller_manager"]),
    ])