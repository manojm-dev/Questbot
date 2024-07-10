import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('questbot_simulation').find('questbot_simulation')
    description_share = FindPackageShare('questbot_description').find('questbot_description')
    gazebo_share = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Files paths 
    default_world_path = os.path.join(pkg_share, 'world/cones.world')

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

    # Launch display configuration
    start_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'launch', 'display.launch.py')),
        launch_arguments={'use_sim_time'    : 'true',
                          'use_jsp'         : 'false',
                          'jsp_gui'         : 'false',
                          'use_rviz'        : 'true',
                          'use_rqt'         : 'false' 
                          }.items()
    )


    # Open simulation environment
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')), 
        launch_arguments={'world': world}.items()
    )
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzclient.launch.py'))
    )
    spawn_entity_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        name="spawn_entity",
        output='screen',
        arguments=['-topic', 'robot_description', '-entity',  'questbot_2wd', '-z', '0.5'],
        parameters=[{
            'use_sim_time': use_sim_time
            }]
    )


    return LaunchDescription(
        declare_arguments + [
            start_description,
            start_gazebo_server,
            start_gazebo_client,
            spawn_entity_node
        ] 
    )