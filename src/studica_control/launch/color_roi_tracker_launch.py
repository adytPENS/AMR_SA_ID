"""Start Gemini E plus interactive HSV ROI tracker."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('studica_control')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'orbbec_gemini_e_launch.py'))),
        Node(
            package='studica_control',
            executable='color_roi_tracker.py',
            name='color_roi_tracker',
            output='screen',
            parameters=[os.path.join(share, 'config', 'color_roi_tracker.yaml')],
        ),
    ])
