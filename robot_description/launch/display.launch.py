import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare(package='robot_description').find('robot_description')

    # File paths
    default_model_path = os.path.join(pkg_share, 'description/robot.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz/gazebo.rviz')


    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # Launch configuration with file paths
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config = LaunchConfiguration('rviz_config')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='jsp_gui', 
            default_value='false',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to open RViz or Not'
        ),
        DeclareLaunchArgument(
            name='urdf_model',
            default_value=default_model_path,
            description='Absolute path of robot URDF file'
        ),
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=rviz_config_path,
            description='Absolute path of RViz config file'
        )
    ]


    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model]) 
            }]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(jsp_gui)
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(jsp_gui)
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )


    return LaunchDescription(
        declare_arguments + [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
        ]
    )