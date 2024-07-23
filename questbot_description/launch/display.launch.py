import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare(
        package='qestbot_description').find('questbot_description')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_jsp = LaunchConfiguration('use_jsp')
    jsp_gui = LaunchConfiguration('jsp_gui')

    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='use_jsp',
            default_value='true',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher'
        ),
        DeclareLaunchArgument(
            name='jsp_gui',
            default_value='false',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'
        ),
    ]

    # Start robot state publisher
    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time':   use_sim_time,
            'use_gazebo':   'true',
            'use_gzsim':   'false',
        }.items()
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(use_jsp)
    )

    # Joint state publisher gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(jsp_gui)
    )

    # Data visualizations
    start_visualization_tools = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_share, 'launch', 'visualize.launch.py'))
    )

    return LaunchDescription(
        declare_arguments + [
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            start_visualization_tools
        ]
    )