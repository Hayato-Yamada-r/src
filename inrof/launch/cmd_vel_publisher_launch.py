import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inrof',
            executable='cmd_vel_publisher',
            name='cmd_vel_publisher',
            output='screen'
        )
    ])