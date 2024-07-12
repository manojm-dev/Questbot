import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
                          'use_gazebo'      : 'false',
                          'use_gzsim'       : 'true',
                          'use_jsp'         : 'false',
                          'jsp_gui'         : 'false',
                          'use_rviz'        : 'true',
                          'use_rqt'         : 'false' 
                          }.items()
    )


    # Open simulation environment
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args':'',
                          'worlds': world}.items()
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
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
            start_description,
            gz_sim,
            bridge
        ] 
    )