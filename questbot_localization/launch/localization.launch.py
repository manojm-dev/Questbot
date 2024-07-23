import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directories
    pkg_share = FindPackageShare(
        'questbot_localization').find('questbot_localization')

    # File paths
    default_ekf_config = os.path.join(pkg_share, 'config/ekf.yaml')

    # Launch configuration variables with default values
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration with file paths
    ekf_config = LaunchConfiguration('ekf_config')

    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='ekf_config',
            default_value=default_ekf_config,
            description='Absolute path of ekf config file'
        ),
    ]

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        declare_arguments + [
            robot_localization_node
        ]
    )
