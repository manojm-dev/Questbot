import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('questbot_gazebo').find('questbot_gazebo')

    # Launch configuration with file paths
    headless = LaunchConfiguration('headless')


    declare_arguments = [

        DeclareLaunchArgument(
            name='headless',
            default_value='False',
            description='Set headless(no gui) mode'
        )
    ]


    launch_gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gzsim.launch.py')),
        launch_arguments={
            'headless'  : headless,
        }.items()
    )

    launch_robot_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn_gz.launch.py')),
        launch_arguments={
            'robot_name': 'quesbot_2wd',
            'spawn_x'   : '1.0',
            'spawn_y'   : '1.0',
            'spawn_z'   : '0.5',
            'spawn_yaw' : '0.0'
        }.items()
    )


    return LaunchDescription(
        declare_arguments + [
            launch_gazebo_sim,
            launch_robot_spawner
        ]
    )