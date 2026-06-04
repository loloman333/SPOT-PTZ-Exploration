#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ptz_exploration_core.utils.ros import handle_config_file

# Package Info
PKG_NAME = 'ptz_exploration_spot'
PKG_PATH = get_package_share_directory(PKG_NAME)
DEFAULT_CONFIG_PATH = os.path.join(PKG_PATH, 'config', 'config.yaml')

def launch_setup(context, *args, **kwargs):

    # Load config    
    config, config_path, actions_to_start = handle_config_file(context, PKG_PATH)
    
    # Start Spot drivers
    actions_to_start.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('our_launchers'),
                    'launch',
                    'spotty.launch.py'
                )
            ),
            launch_arguments={
                'config_file': config_path,
                'image_pub_config': os.path.join(PKG_PATH, 'config', 'image_pub.yaml'),
                'rviz_config_file': os.path.join(PKG_PATH, 'config', 'spot.rviz'),
            }.items()
        )
    )
    
    # PTZ Wrapper Node
    actions_to_start.append(
        Node(
            package=PKG_NAME,
            executable='ptz_wrapper',
            name='ptz_wrapper',
            parameters=[config_path],
            output='screen'
        )
    )
    
    return actions_to_start

def generate_launch_description():
    return LaunchDescription([

        # # Force local-only DDS traffic.
        # SetEnvironmentVariable(
        #     name='ROS_LOCALHOST_ONLY',
        #     value='1'
        # ),

        SetEnvironmentVariable(
            name='FASTRTPS_DEFAULT_PROFILES_FILE',
            value='/home/appuser/ros2_ws/src/my_packages/ptz_exploration_spot/config/custom_dds_profile.xml'
        ),

        # Declare config file argument
        DeclareLaunchArgument(
            'config_file',
            default_value=DEFAULT_CONFIG_PATH,
            description='Path to the configuration file'
        ),
        
        # Register OpaqueFunction to run at launch time
        OpaqueFunction(function=launch_setup)
    ])