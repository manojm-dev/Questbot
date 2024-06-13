import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]

def generate_launch_description():

    # Package Name
    pkg_name = 'robot_dd'

    # Packages Used
    pkg_robot_dd = get_package_share_directory(pkg_name)
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')

    # File Paths
    rviz_subpath = 'config/view_robot.rviz'
    world_subpath = 'world/willowgarage.world'
    xacro_subpath = 'description/robot.urdf.xacro'

    # Files Directories
    rviz_file = os.path.join(pkg_robot_dd, rviz_subpath)
    world_file = os.path.join(pkg_robot_dd, world_subpath)
    xacro_file = os.path.join(pkg_robot_dd, xacro_subpath)
    urdf_file = xacro.process_file(xacro_file).toxml()
    bridge_params = os.path.join(pkg_robot_dd,'config','gz_bridge_params.yaml')


    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_robot_dd, 'worlds')]
    )

    # robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_dd,'launch','rsp.launch.py'))
    )

    # Gazebo ros bridge
    gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # start gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')), 
        launch_arguments={'gz_args': ['-r -s -v4 ','empty.sdf'], 
                          'on_exit_shutdown': 'true',
                          'use_sim_time':'true'}.items()
    )
    
    # start gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-name', 'robot_dd',
            '-topic', '/robot_description'
        ]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_file]
    )
    
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_cont"]
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"]
    # )
    

    ld = LaunchDescription()

    # ld.add_action(set_gazebo_model_path)
    # ld.add_action(declare_world)
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo_ros_bridge)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_entity)
    ld.add_action(rviz)
    # ld.add_action(diff_drive_spawner)
    # ld.add_action(joint_broad_spawner)

    return ld