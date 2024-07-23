import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare(
        package='qestbot_description').find('questbot_description')

    # File paths
    default_model_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')

    # Launch configuration variables (used to modify at launch time)
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_gzsim = LaunchConfiguration('use_gzsim')

    # Launch configuration with file paths (used to modify at launch time)
    urdf_model = LaunchConfiguration('urdf_model')

    # Default launch configuration arguments
    declare_arguments = [
        DeclareLaunchArgument(
            name='urdf_model',
            default_value=default_model_path,
            description='Absolute path of robot URDF file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='use_gazebo',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo classic'
        ),
        DeclareLaunchArgument(
            name='use_gzsim',
            default_value='false',
            choices=['true', 'false'],
            description='Use Gazebo Sim'
        ),
    ]

    # Start robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ',           urdf_model,
                ' use_gazebo:=',    use_gazebo,
                ' use_gzsim:=',     use_gzsim
            ])
        }]
    )

    return LaunchDescription(
        declare_arguments + [
            robot_state_publisher_node,
        ]
    )
