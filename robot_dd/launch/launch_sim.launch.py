import os


from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

  pkg_name = 'robot_dd'

  robot_state_publisher = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource([os.path.join(
                                  get_package_share_directory(pkg_name),'launch','rsp.launch.py'
                                  )]), launch_arguments={'use_sim_time':'true'}.items()
  )

  gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]) 
  )

  spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic', 'robot_description',
                                 '-entity', 'robot_dd',],
                      output='screen')
  
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
  
  return LaunchDescription([
    robot_state_publisher,
    gazebo,
    spawn_entity,
    diff_drive_spawner,
    joint_broad_spawner
  ])