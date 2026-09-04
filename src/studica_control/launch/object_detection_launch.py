"""Start the Gemini E and lightweight RGB-D object detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('studica_control')
    camera_launch = os.path.join(share, 'launch', 'orbbec_gemini_e_launch.py')
    detector_config = os.path.join(share, 'config', 'object_detection.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch)),
        Node(
            package='studica_control',
            executable='rgbd_object_detector.py',
            name='rgbd_object_detector',
            output='screen',
            parameters=[detector_config],
        ),
    ])
