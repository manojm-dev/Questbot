import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('questbot_gazebo').find('questbot_gazebo')
    description_share = FindPackageShare('questbot_description').find('questbot_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    # Files paths 
    default_world_path = os.path.join(pkg_share, 'empty.sdf')

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [

        DeclareLaunchArgument(
            name='robot_name', 
            description='Name of the robot'
        ),

        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='0.0',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
            
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn orientation'
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

    # Gazebo Sim Bridge
    gz_bridge = Node(
        name="ros_gz_bridge",
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml'),
        }],
        output='screen'
    )

    spawn_entity_node = Node(
        package='ros_gz_sim', 
        executable='create',
        name='spawn_entity',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-timeout', '120.0',
            '-topic', 'robot_description', 
            '-entity', robot_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
            ],
        
        output='screen',
    )

    return LaunchDescription(
        declare_arguments + [
            robot_state_publisher,
            gz_bridge,
            spawn_entity_node
        ] 
    )