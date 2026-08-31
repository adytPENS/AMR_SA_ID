import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('studica_control')
    odom_params = LaunchConfiguration('odom_params')
    slam_params = LaunchConfiguration('slam_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_params',
            default_value=os.path.join(
                package_share, 'config', 'wheel_odometry.yaml')),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(
                package_share, 'config', 'slam_toolbox.yaml')),
        Node(
            package='studica_control',
            executable='wheel_odometry.py',
            name='wheel_odometry',
            output='screen',
            parameters=[odom_params]),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]),
    ])
