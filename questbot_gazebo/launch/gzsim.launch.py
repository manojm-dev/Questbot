import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('questbot_gazebo').find('questbot_gazebo')
    description_share = FindPackageShare('questbot_description').find('questbot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    # File paths
    #world = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    world = 'empty.sdf'

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    # Launch Arguments (used to modify at launch time)
    declare_arguments = [

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use Gazebo Clock',
        ),

        DeclareLaunchArgument(
            name='headless',
            default_value='False', 
            description='Headless mode or not'
        ),
    ]


    # Start Gazebo sim
    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world}.items(),
    )


    return LaunchDescription(
        declare_arguments + [
            start_gz_sim
        ] 
    )