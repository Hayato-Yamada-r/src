#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Gazeboの起動
        ExecuteProcess(
            cmd=['gz', 'sim', '/home/hak/ros2_ws/src/inrof/sdf/simple_robot.sdf'],
            output='log',
            log_cmd=True
        ),

        # ブリッジの起動
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='log',
            log_cmd=True
        ),

        # teleop_twist_keyboardの起動
        ExecuteProcess(
            cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
            output='screen',
            prefix='xterm -e'
        ),

    ])