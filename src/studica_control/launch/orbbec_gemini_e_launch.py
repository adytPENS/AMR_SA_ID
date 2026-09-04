"""Launch the Studica/Orbbec Gemini E from a readable YAML configuration."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_launch_value(value):
    """ROS launch arguments are strings, including YAML booleans."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _include_camera(context):
    config_path = LaunchConfiguration('camera_config').perform(context)
    with open(config_path, encoding='utf-8') as config_file:
        document = yaml.safe_load(config_file) or {}

    camera_config = document.get('camera')
    if not isinstance(camera_config, dict):
        raise RuntimeError(
            f"{config_path} must contain a top-level 'camera' mapping")

    upstream_launch = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'gemini_e.launch.py',
    )
    launch_arguments = {
        key: _as_launch_value(value)
        for key, value in camera_config.items()
        if value is not None
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_launch),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('studica_control'),
        'config',
        'orbbec_gemini_e.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config',
            default_value=default_config,
            description='Path to the Gemini E camera configuration YAML file.',
        ),
        OpaqueFunction(function=_include_camera),
    ])
