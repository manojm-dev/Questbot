import os

from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_dd').find('robot_dd')
    model_path = os.path.join(pkg_share, 'description/robot.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz/display.rviz')

    launch_arguments = [
        DeclareLaunchArgument(
            name=arg_name,
            default_value=arg_default,
            description=arg_description
        )
        for arg_name, arg_default, arg_description in [
            ('gui', 'True', 'Flag to enable joint_state_publisher_gui'),
            ('model', model_path, 'Absolute path to robot URDF file'),
            ('rvizconfig', rviz_config_path, 'Absolute path to RViz config file')
        ]
    ]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model_path])}],
        condition=conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=conditions.IfCondition(LaunchConfiguration('gui'))
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
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
        ]
    )