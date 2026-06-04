#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ptz_exploration_core.utils.ros import handle_config_file

# Package Info
PKG_NAME = 'ptz_exploration_core'
PKG_PATH = get_package_share_directory(PKG_NAME)
DEFAULT_CONFIG_PATH = os.path.join(PKG_PATH, 'config', 'config_sim.yaml')

def launch_setup(context, *args, **kwargs):

    # Load config    
    config, config_path, actions_to_start = handle_config_file(context, PKG_PATH)
    
    # Octomap Server
    if config['launch_octomap']:
        
        actions_to_start.append(
            Node(
                package=PKG_NAME,
                executable='cloud_in', 
                name='cloud_in', 
                parameters=[config_path], 
                output='screen' 
            )
        )
        
        actions_to_start.append(
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                prefix=['bash -c \'exec "$0" "$@" >/dev/null 2>&1\''],
                parameters=[config_path],
                output='log',
                arguments=[
                    '--ros-args',
                    '--disable-stdout-logs',
                    '--disable-rosout-logs',
                    '--log-level',
                    'fatal'
                ]
            )
        )
    
    # Nav2 Bringup
    if config['launch_nav2']:
        
        nav2_config = config['nav2_config']
        nav2_params_file = os.path.join(PKG_PATH, 'config', nav2_config)
                
        actions_to_start.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    )
                ),
                launch_arguments={
                    'params_file': nav2_params_file,
                    'use_sim_time': str(config['use_sim_time']).lower(),
                    'autostart': 'true',
                    'namespace': config['nav2_namespace'],
                }.items()
            )
        )
    
    # YOLO Detector for 360 Camera
    if config['launch_detector']:
        actions_to_start.append(
            Node(
                package=PKG_NAME,
                executable='detector',
                name='detector',
                parameters=[config_path],
                output='screen'
            )
        )
    
    # Gazer Node
    if config['launch_gazer']:
        actions_to_start.append(
            Node(
                package=PKG_NAME,
                executable='gazer',
                name='gazer',
                parameters=[config_path],
                output='screen'
            )
        )
    
    # Projector & Optimizer Nodes
    if config['launch_optimizer']:
        actions_to_start.append(
            Node(
                package=PKG_NAME,
                executable='projector',
                name='projector',
                parameters=[config_path],
                output='screen'
            )
        )
        
        actions_to_start.append(
            Node(
                package=PKG_NAME,
                executable='optimizer',
                name='optimizer',
                parameters=[config_path],
                output='screen',
                # arguments=['--ros-args', '--log-level', 'optimizer:=DEBUG']
            )
        )
    
    # Explorer Node
    if config['launch_explorer']:
        actions_to_start.append(
            Node(
                package=PKG_NAME,
                executable='explorer',
                name='explorer',
                parameters=[config_path],
                output='screen'
            )
        )

    # Collector Node
    if config.get('launch_collector', False):
        actions_to_start.append(
            Node(
                package=PKG_NAME,
                executable='collector',
                name='collector',
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
        
        # Declare config file argument
        DeclareLaunchArgument(
            'config_file',
            default_value=DEFAULT_CONFIG_PATH,
            description='Path to the configuration file'
        ),
        
        # Register OpaqueFunction to run at launch time
        OpaqueFunction(function=launch_setup)
    ])