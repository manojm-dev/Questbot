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
    pkg_share = FindPackageShare('questbot_description').find('questbot_description')
    gazebo_share = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Files paths 
    default_model_path = os.path.join(pkg_share, 'description/robot.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz/gazebo.rviz')
    rqt_perspective_path = os.path.join(pkg_share, 'rviz/rqt_nodes.perspective')
    default_world_path = os.path.join(pkg_share, 'world/empty.world')
    ekf_config_path = os.path.join(pkg_share, 'config/ekf.yaml')

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Launch configuration with file paths
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config = LaunchConfiguration('rviz_config')
    rqt_perspective = LaunchConfiguration('rqt_perspective')
    world = LaunchConfiguration('world')
    efk_config = LaunchConfiguration('ekf_config')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='urdf_model',
            default_value=default_model_path,
            description='Absolute path of robot URDF file'
        ),
        DeclareLaunchArgument(
            name='ekf_config',
            default_value=ekf_config_path,
            description='Extended kalman filer YAML config file'
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
            name='rqt_perspective',
            default_value=rqt_perspective_path,
            description='Absolute path of Rqt gui perspective file'
        ),
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path, 
            description='Absolute path of gazebo WORLD file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        )
    ]

    # Start robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model])
            }]
    )

    # Localization

    ## Extended kalman filter
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[
           efk_config, 
           {'use_sim_time': use_sim_time}]
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

    # Data visualizations

    ## Open RViz
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

    ## Open rqt visulaizer
    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_gui',
        output='screen',
        arguments=['--perspective-file', rqt_perspective]
    )


    return LaunchDescription(
        declare_arguments + [
            robot_state_publisher_node,
            start_gazebo_server,
            start_gazebo_client,
            spawn_entity_node,
            robot_localization_node,
            rviz_node,
            rqt_node,
        ] 
    )