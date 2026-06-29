import os
from launch import LaunchDescription
from launch_ros.actions import Node

VENV_SITE_PACKAGES = "/home/zarus101/witsense/moveit-ros/.yolo_moveit/lib/python3.12/site-packages"

def generate_launch_description():
    detector = Node(
        package="object_detection_yolo", executable="yolo_3d_detector",
        output="screen",
        parameters=[{
        "use_sim_time": True,
        "optical_frame": "front_stereo_camera_left",   # Isaac is already optical
        "detect_all": False,
        "dump_frames": 0,
        }],
        additional_env={
            "PYTHONPATH": VENV_SITE_PACKAGES + os.pathsep + os.environ.get("PYTHONPATH", "")
        },
    )
    return LaunchDescription([detector])