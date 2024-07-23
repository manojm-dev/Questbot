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

    # File paths
    rviz_config_path = os.path.join(pkg_share, 'config/display.rviz')
    rqt_perspective_path = os.path.join(
        pkg_share, 'config/display.perspective')

    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    use_rqt = LaunchConfiguration('use_rqt')

    # Launch configuration with file paths
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    rqt_perspective = LaunchConfiguration('rqt_perspective')

    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to open RViz or Not'
        ),
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=rviz_config_path,
            description='Absolute path of RViz config file'
        ),
        DeclareLaunchArgument(
            name='use_rqt',
            default_value='false',
            choices=['true', 'false'],
            description='Flag to enable rqt gui'
        ),
        DeclareLaunchArgument(
            name='rqt_perspective',
            default_value=rqt_perspective_path,
            description='Absolute path of Rqt gui perspective file'
        ),
    ]

    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_rviz)
    )

    # Start rqt visulaizer
    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_gui',
        output='screen',
        arguments=['--perspective-file', rqt_perspective],
        condition=IfCondition(use_rqt)
    )

    return LaunchDescription(
        declare_arguments + [
            rviz_node,
            rqt_node
        ]
    )
