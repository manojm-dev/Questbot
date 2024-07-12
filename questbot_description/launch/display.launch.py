import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare(package='qestbot_description').find('questbot_description')

    # File paths
    default_model_path = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'config/display.rviz')
    rqt_perspective_path = os.path.join(pkg_share, 'config/display.perspective')


    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_gzsim = LaunchConfiguration('use_gzsim')
    use_jsp = LaunchConfiguration('use_jsp')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rqt = LaunchConfiguration('use_rqt')

    # Launch configuration with file paths
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config = LaunchConfiguration('rviz_config')
    rqt_perspective = LaunchConfiguration('rqt_perspective')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='urdf_model',
            default_value=default_model_path,
            description='Absolute path of robot URDF file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='use_gazebo', 
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo classic'
        ),
        DeclareLaunchArgument(
            name='use_gzsim', 
            default_value='false',
            choices=['true', 'false'],
            description='Use Gazebo Sim'
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
            name='use_jsp', 
            default_value='true',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher'
        ),
        DeclareLaunchArgument(
            name='jsp_gui', 
            default_value='false',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='use_rqt',
            default_value='false',
            choices=['true', 'false'],
            description='Flag to enable rqt gui'
        ),
        DeclareLaunchArgument(
            name='rqt_perspective',
            default_value=rqt_perspective_path,
            description='Absolute path of Rqt gui perspective file'
        ),
    ]


    # Start robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model,
                                          ' use_gazebo:=', use_gazebo,
                                          ' use_gzsim:=', use_gzsim
                                          ])
            }]
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(use_jsp)
    )

    # Joint state publisher gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(jsp_gui)
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
        arguments=['--perspective-file', rqt_perspective],
        condition=IfCondition(use_rqt)
    )



    return LaunchDescription(
        declare_arguments + [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        rqt_node
        ]
    )