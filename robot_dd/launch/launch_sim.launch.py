import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

  # Pkg name & rviz file path
  pkg_name = 'robot_dd'
  rviz_subpath = 'config/view_robot.rviz'
  world_subpath = 'world/willowgarage.world'


  # Get pkg shares dir
  pkg_robot_dd = get_package_share_directory(pkg_name)
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


  # Get files
  rviz_file = os.path.join(pkg_robot_dd, rviz_subpath)
  world_file = os.path.join(pkg_robot_dd, world_subpath)


  # Launch configuration variable specific for simulations
  world = LaunchConfiguration('world')


  # Declaring Launch arguments
  declare_world = DeclareLaunchArgument(
    name='world',
    default_value=world_file,
    description='Full path to the world model file to load')
  

  # robot state publisher
  robot_state_publisher = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_robot_dd,'launch','rsp.launch.py')), 
    launch_arguments={'use_sim_time':'true'}.items()
  )

  # start gazebo server
  gazebo_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')), 
    launch_arguments={'world': world_file}.items()
  )
  
  # start gazebo client
  gazebo_client = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
  )

  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic', 'robot_description',
                                 '-entity', 'robot_dd',],
                      output='screen')
  
  rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    parameters=[{'use_sim_time': True}],
    arguments=['-d', rviz_file]
  )
  
  diff_drive_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["diff_cont"]
  )

  joint_broad_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broad"]
  )
  

  ld = LaunchDescription()

  ld.add_action(declare_world)
  ld.add_action(robot_state_publisher)
  ld.add_action(gazebo_server)
  ld.add_action(gazebo_client)
  ld.add_action(spawn_entity)
  ld.add_action(rviz)
  ld.add_action(diff_drive_spawner)
  ld.add_action(joint_broad_spawner)

  return ld