import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('robot_dd').find('robot_dd')
    model_path = os.path.join(pkg_share, 'description/robot.xacro')
    world_path = os.path.join(pkg_share,'world/empty.world')
    rviz_config_path = os.path.join(pkg_share, 'rviz/gazebo.rviz')

    launch_arguments = [
        DeclareLaunchArgument(
            name=arg_name,
            default_value=arg_default,
            description=arg_description
        )
        for arg_name, arg_default, arg_description in [
            ('model', model_path, 'Absolute path to robot URDF file'),
            ('world', world_path, 'Absolute paht to gazebo WORLD file'),
            ('rvizconfig', rviz_config_path, 'Absolute path to RViz config file')
        ]
    ]
    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    gazebo_server_node = Node(
        name='gazebo_server',
        executable='gzserver',
        arguments=['-d',LaunchConfiguration('world')],
        output='screen',
    )
    gazebo_client_node = Node(
        name='gazebo_client',
        executable='gzclient',
        output='screen',
    )
    spawn_entity_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'robot_dd',],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription(
        launch_arguments + [
            robot_state_publisher_node,
            gazebo_server_node,
            gazebo_client_node,
            spawn_entity_node,
            rviz_node,
        ]
    )