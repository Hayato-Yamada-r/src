import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # slam_toolboxの共有ディレクトリ（ROS2ディストリビューションに合わせ変更）
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    
    # ロボットのモデル（SDF）が含まれるパッケージのディレクトリ
    inrof_share = get_package_share_directory('inrof')
    
    # RViz用の設定ファイル（必要なら）
    rviz_config = os.path.join(inrof_share, 'launch', 'rviz_config.rviz')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        
        # slam_toolbox のオンライン非同期SLAMを起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': os.path.join(
                    slam_toolbox_share, 'config', 'mapper_params_online_async.yaml'
                )
            }.items()
        ),
        
        # ロボットのSDFモデルをGazeboで起動している場合、ライダーセンサやTFが正しく発行されるようにする
        # Static transform: lidar_link -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_base_link',
            arguments=['0', '0', '-0.15', '0', '0', '0', 'lidar_link', 'base_link'],
            output='screen'
        ),
        
        # RViz2を起動する（オプション）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])