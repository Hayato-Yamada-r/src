import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('inrof')
    # Assuming your robot description is defined using a xacro in the urdf folder
    xacro_file = os.path.join(pkg_share, 'urdf', 'simple_robot.xacro')
    rviz_config_file = os.path.join(pkg_share, 'launch', 'rviz_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        # Node to publish the robot_state (TF information)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': Command(['xacro ', xacro_file])
            }]
        ),
        # RViz2 Node to display the robot model in RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ])