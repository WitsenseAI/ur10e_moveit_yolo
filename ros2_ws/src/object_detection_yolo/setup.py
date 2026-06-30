import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'object_detection_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zarus101',
    maintainer_email='raz.thapaliya600@gmail.com',
    description='YOLO 3D detection and MoveIt pick-and-place for the UR10e in Isaac Sim',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_3d_detector = object_detection_yolo.yolo_3d_detector:main',
            'pick_place = object_detection_yolo.pick_place:main',
            'move_to_pose = object_detection_yolo.move_to_pose:main'
        ],
    },
)
