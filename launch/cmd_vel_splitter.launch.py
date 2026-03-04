import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_cmd_vel_splitter'),
        'config',
        'cmd_vel_splitter.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_cmd_vel_splitter',
            executable='cmd_vel_splitter',
            name='cmd_vel_splitter',
            parameters=[config]
        )
    ])
