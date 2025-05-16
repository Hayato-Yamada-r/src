import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'inrof', 'launch', 'rviz_config.rviz'
    )
    return LaunchDescription([
        Node(
            package='inrof',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher',
            output='screen'
        ),
        Node(
            package='inrof',
            executable='auto_drive',
            name='auto_drive',
            output='screen'
        ),
        Node(
            package='inrof',
            executable='twist_marker',
            name='twist_marker',
            output='screen'
        ),
        # RViz2の起動（rviz_config.rvizがあればそれを使用）
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])