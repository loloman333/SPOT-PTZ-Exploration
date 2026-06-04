#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription
from launch_ros.actions import Node
from ptz_exploration_core.utils.ros import handle_config_file

# Package Info
PKG_NAME = 'ptz_exploration_sim'
PKG_PATH = get_package_share_directory(PKG_NAME)
DEFAULT_CONFIG_PATH = os.path.join(PKG_PATH, 'config', 'config.yaml')

def launch_setup(context, *args, **kwargs):

    # Load config    
    config, config_path, actions_to_start = handle_config_file(context, PKG_PATH)
    
    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'robot',
            '-x', str(config['initial_pose']['x']),  
            '-y', str(config['initial_pose']['y']),
            '-z', str(config['initial_pose']['z']),
            '-R', str(config['initial_pose']['roll']),
            '-P', str(config['initial_pose']['pitch']),
            '-Y', str(config['initial_pose']['yaw'])
        ],
        output='screen'
    )
    actions_to_start.append(spawn_robot)
    gazebo_ros2_mappings = []

    # Robot State Publisher
    xacro_file = os.path.join(PKG_PATH, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            '360_camera': str(config['simulate_360_camera']).lower(),
            'ptz_camera': str(config['simulate_ptz']).lower(),
        }    
    )
    robot_desc = robot_description_config.toxml()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        output='screen'
    )
    actions_to_start.append(robot_state_publisher)
    gazebo_ros2_mappings += [
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
    ]
    
    # map is odom transform
    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'tf_map_to_odom',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            f'map',
            f'odom',
        ],
        output='screen'
    )
    actions_to_start.append(map_tf)
    
    # Delay until after spawn/joint_states to ensure TF is available
    delayed_list = []

    # Launch RViz 
    if config['launch_rviz']:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(PKG_PATH, 'config', 'sim.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        delayed_list.append(rviz_node)
    
    # PTZ Manager
    if config['simulate_ptz']:
        ptz_manager = Node(
            package=PKG_NAME,
            executable='ptz_simulator',
            name='ptz_simulator',
            parameters=[config_path],
            output='screen'
        )
        delayed_list.append(ptz_manager)
        
        gazebo_ros2_mappings += [
            '/ptz_cam/raw_full/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/ptz_cam/raw_full/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/ptz_cam/cmd_pan@std_msgs/msg/Float64@gz.msgs.Double',
            '/ptz_cam/cmd_tilt@std_msgs/msg/Float64@gz.msgs.Double'
        ]
        
        map_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'tf_ptz_cam_static',
            arguments=[
            '0', '0', '0', '-1.5708', '0', '-1.5708',
            'ptz_cam_link',
            'ptz_cam_optical_link',
            ],
            output='screen'
        )
        delayed_list.append(map_tf)

    # 360 Camera Array
    if config['simulate_360_camera']: 
        for cam in ['front', 'rear', 'left', 'right']:

            gazebo_ros2_mappings += [
                f'/{cam}_cam_360/depth/image@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/{cam}_cam_360/depth/image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            ]
            
            map_tf = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'tf_{cam}_cam_static',
                arguments=[
                    '0', '0', '0', '0', '0', '0',
                    f'{cam}_cam_360_link',
                    f'robot/base_link/{cam}_cam_360_depth_sensor',
                ],
                output='screen'
            )
            delayed_list.append(map_tf)
        
    # ROS2 Bridge 
    gazebo_ros2_mappings += [
        f'/world/{config["world_name"]}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
    ]       
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=gazebo_ros2_mappings,
        output='screen'
    )
    actions_to_start.append(bridge)
        
    world_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', config['world_name'], 'map'],
        output='screen'
    )
    actions_to_start.append(world_to_map_tf)
     
    # Delay Nodes for after Spawn
    if len(delayed_list) > 0:
        
        wait_for_joints = ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/joint_states', '--once'],
            name='wait_for_joints',
            output='log'
        )
        actions_to_start.append(wait_for_joints)
        
        delayed_nodes = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_joints,
                on_exit=delayed_list
            )
        )
        actions_to_start.append(delayed_nodes)

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