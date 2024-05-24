import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

  # pkg name and path to xacro file
  pkg_name = 'robot_dd'
  file_subpath = 'description/robot.urdf.xacro'

  # use xacro to process the file
  xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
  robot_description_raw = xacro.process_file(xacro_file).toxml()

  # rsp
  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[{'robot_description': robot_description_raw}]
  )

  return LaunchDescription([
    robot_state_publisher_node
  ])