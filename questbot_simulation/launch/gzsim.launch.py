import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('questbot_simulation').find('questbot_simulation')
    description_share = FindPackageShare('questbot_description').find('questbot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    # Files paths 
    default_world_path = os.path.join(pkg_share, 'empty.sdf')

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration with file paths
    world = LaunchConfiguration('world')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path, 
            description='Absolute path of gazebo WORLD file'
        ),
    ]

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time'    : 'true',
                          'use_gazebo'      : 'false',
                          'use_gzsim'       : 'true',
                          }.items()
    )

    # Start Simulation Environment
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', 'empty.sdf'],
        output='screen'
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    gz_bridge = Node(
        name="ros_gz_bridge",
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription(
        declare_arguments + [
            robot_state_publisher,
            gz_sim,
            gz_bridge
        ] 
    )