# Copyright (c) 2023-2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

import os

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from synchros2.launch.actions import DeclareBooleanLaunchArgument, convert_to_bool

from spot_common.launch.spot_launch_helpers import (
    IMAGE_PUBLISHER_ARGS,
    declare_image_publisher_args,
    get_name_and_prefix,
    spot_has_arm,
    substitute_launch_parameters,
    get_ros_param_dict,
)

THIS_PACKAGE = "our_launchers"


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:

    config_file = LaunchConfiguration("config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    spot_name_arg = LaunchConfiguration("spot_name")
    tf_prefix_arg = LaunchConfiguration("tf_prefix")
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    mock_enable = IfCondition(LaunchConfiguration("mock_enable", default="False")).evaluate(context)
    robot_description_package = LaunchConfiguration("robot_description_package").perform(context)
    controllable = convert_to_bool("controllable", LaunchConfiguration("controllable").perform(context))

    # if config_file has been set (and is not the default empty string) and is also not a file, do not launch anything.
    config_file_path = config_file.perform(context)
    print("config_file_path: ", config_file_path)
    if (config_file_path != "") and (not os.path.isfile(config_file_path)):
        raise FileNotFoundError("Configuration file '{}' does not exist!".format(config_file_path))

    substitutions = {
        "spot_name": spot_name_arg,
        "frame_prefix": tf_prefix_arg,
    }
    configured_params = substitute_launch_parameters(config_file_path, substitutions, context)
    spot_name, tf_prefix = get_name_and_prefix(configured_params)

    if mock_enable:
        mock_has_arm = IfCondition(LaunchConfiguration("mock_has_arm")).evaluate(context)
        has_arm = mock_has_arm
    else:
        has_arm = spot_has_arm(config_file_path=config_file.perform(context))

    robot_description_pkg_share = FindPackageShare(robot_description_package).find(robot_description_package)

    spot_driver_params = {
        "mock_enable": mock_enable,
    }

    if mock_enable:
        mock_spot_driver_params = {"mock_has_arm": mock_has_arm}
        # Merge the two dicts
        spot_driver_params = {**spot_driver_params, **mock_spot_driver_params}

    if controllable:
        spot_driver_params.update(
            {
                "leasing_mode": "proxied",
                "use_take_lease": False,
                "get_lease_on_action": True,
            }
        )

    set_use_sim_time = SetParameter(
        name="use_sim_time",
        value=LaunchConfiguration("use_sim_time"),
    )
    ld.add_action(set_use_sim_time)

    spot_driver_node = Node(
        package="spot_driver",
        executable="spot_ros2",
        name="spot_ros2",
        output="screen",
        parameters=[configured_params, spot_driver_params],
        namespace=spot_name,
    )
    ld.add_action(spot_driver_node)

    spot_lease_manager_node = Node(
        package="spot_driver",
        executable="lease_manager_node",
        name="lease_manager_node",
        output="screen",
        parameters=[configured_params],
        namespace=spot_name,
        condition=IfCondition(LaunchConfiguration("controllable")),
    )
    ld.add_action(spot_lease_manager_node)

    kinematic_node = Node(
        package="spot_driver",
        executable="spot_inverse_kinematics_node",
        output="screen",
        parameters=[configured_params],
        namespace=spot_name,
    )
    ld.add_action(kinematic_node)

    object_sync_node = Node(
        package="spot_driver",
        executable="object_synchronizer_node",
        output="screen",
        parameters=[configured_params],
        namespace=spot_name,
    )
    ld.add_action(object_sync_node)

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([robot_description_pkg_share, "urdf", "spot.urdf.xacro"]),
            " ",
            "arm:=",
            TextSubstitution(text=str(has_arm).lower()),
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    # Publish frequency of the robot state publisher defaults to 20 Hz, resulting in slow TF lookups.
    # By ignoring the timestamp, we publish a TF update in this node every time there is a joint state update (50 Hz).
    robot_description_params = {"robot_description": robot_description, "ignore_timestamp": True}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_params],
        namespace=spot_name,
    )
    ld.add_action(robot_state_publisher)

    spot_robot_state_publisher = Node(
        package="spot_driver",
        executable="state_publisher_node",
        output="screen",
        parameters=[configured_params],
        namespace=spot_name,
    )
    ld.add_action(spot_robot_state_publisher)

    spot_alert_node = Node(
        package="spot_driver",
        executable="spot_alerts",
        name="spot_alerts",
        output="screen",
        namespace=spot_name,
    )
    ld.add_action(spot_alert_node)

    rviz_launch_condition = IfCondition(LaunchConfiguration("launch_rviz")).evaluate(context)
    if 'launch_rviz' in configured_params:
        rviz_launch_condition = configured_params['launch_rviz']
    if 'rviz_config_file' in configured_params:
        rviz_config_file = configured_params['rviz_config_file']
    if rviz_launch_condition:
        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("spot_driver"), "launch", "rviz.launch.py"])
            ),
            launch_arguments={
                "spot_name": spot_name,
                "rviz_config_file": rviz_config_file,
                "tf_prefix": tf_prefix,
            }.items(),
        )
        ld.add_action(rviz)

    # Start with the values from LaunchConfigurations (the CLI/defaults)
    # We resolve them to strings/actual values using context.perform()
    final_image_args = {
        key: LaunchConfiguration(key).perform(context)
        for key in ["config_file", "tf_prefix", "spot_name", "use_sim_time"] + IMAGE_PUBLISHER_ARGS
    }

    # Overwrite with values from your YAML file (configured_params) if they exist
    for key in final_image_args.keys():
        if key in configured_params:
            if key in ["tf_prefix", "spot_name"]:
                # For these keys, we want to use the LaunchConfiguration value (which may have been set via CLI) instead of the YAML value
                continue
            # Convert to string because launch_arguments expects strings or substitutions
            final_image_args[key] = str(configured_params[key])

    image_node_launch_condition = IfCondition(LaunchConfiguration("launch_image_publishers")).evaluate(context)

    if 'launch_image_publishers' in configured_params:
        image_node_launch_condition = configured_params['launch_image_publishers']

    # Use this merged dictionary in your IncludeLaunchDescription
    if image_node_launch_condition:
        spot_image_publishers = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("spot_driver"), "launch", "spot_image_publishers.launch.py"])
            ),
            launch_arguments=final_image_args.items(), # .items() converts dict to list of tuples
        )
        ld.add_action(spot_image_publishers)

    spot_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_ros2_control"), "launch", "spot_ros2_control.launch.py"])
        ),
        launch_arguments={
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "config_file": LaunchConfiguration("config_file"),
            "controllers_config": LaunchConfiguration("controllers_config"),
            "spot_name": LaunchConfiguration("spot_name"),
            "hardware_interface": "mock" if mock_enable else "robot",
            "mock_arm": str(mock_enable and has_arm),
            "launch_image_publishers": False,
            "leasing_mode": "proxied",
            "control_only": "True",
            "auto_start": "False",
        }.items(),
        condition=IfCondition(LaunchConfiguration("controllable")),
    )
    ld.add_action(spot_ros2_control)


def generate_launch_description() -> LaunchDescription:
    launch_args = []

    default_config_path = PathJoinSubstitution([
            FindPackageShare(THIS_PACKAGE),
            "config",
            "our_spotty.yaml"
        ])
    
    launch_args.append( 
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_path,
            description="Path to the configuration file"
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "controllable",
            default_value=False,
            description="If true, enable low-level control capabilities",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "controllers_config",
            default_value="",
            description=(
                "If controllable, configuration file for spot_ros2_control controllers. "
                "See spot_ros2_control.launch.py for further reference."
            ),
        ),
    )
    launch_args.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="spotty/",
            description="apply namespace prefix to robot links and joints",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "use_sim_time",
            default_value=False,
            description="If true, use simulation time instead of real time",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "launch_rviz",
            default_value=False,
            description="Choose whether to launch RViz",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="",
            description="RViz config file",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "launch_image_publishers",
            default_value=True,
            description="Choose whether to launch the image publishing nodes from Spot.",
        )
    )
    
    launch_args.append(
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="spot_description",
            description="Package from where the robot model description is. Must have path /urdf/spot.urdf.xacro",
        )
    )

    launch_args += declare_image_publisher_args()

    launch_args.append(
        DeclareLaunchArgument(
            "spot_name",
            default_value="spotty",
            description="Name of Spot"
        )
    )

    ld = LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))

    return ld
