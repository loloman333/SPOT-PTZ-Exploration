#!/usr/bin/env python3
import os
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ptz_exploration_core.utils.ros import handle_config_file

# Package Info
PKG_NAME = 'ptz_exploration_sim'
PKG_PATH = get_package_share_directory(PKG_NAME)
DEFAULT_CONFIG_PATH = os.path.join(PKG_PATH, 'config', 'config.yaml')

def launch_setup(context, *args, **kwargs):

    # Load config    
    config, config_path, actions_to_start = handle_config_file(context, PKG_PATH)
    
    # Gazebo
    world_name = config['world_name']
    world_xacro_file_path = os.path.join(PKG_PATH, 'worlds', f'{world_name}.xacro.sdf')
    world_file_path = os.path.join(PKG_PATH, 'worlds', f'{world_name}.sdf')
    object_config_file_path = os.path.join(PKG_PATH, 'worlds', f'{world_name}_objects.yaml')

    if os.path.exists(world_xacro_file_path):
        object_file_for_xacro = object_config_file_path if os.path.exists(object_config_file_path) else ''
        world_doc = xacro.process_file(
            world_xacro_file_path,
            mappings={
                'objects_file': object_file_for_xacro,
            }
        )

        with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as generated_world_file:
            generated_world_file.write(world_doc.toxml())
            world_file_path = generated_world_file.name

    gazebo_verbosity = config['gazebo_verbosity']
    
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'/opt/ros/humble/share'
    )
    actions_to_start.append(set_ign_resource_path)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file_path} -v {gazebo_verbosity}',
        }.items()
    )
    actions_to_start.append(gazebo)
    
    return actions_to_start

def generate_launch_description():
    return LaunchDescription([
        # Force local-only DDS traffic.
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

