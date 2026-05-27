from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('sealien_mavlink_joy_bridge')
    return LaunchDescription([
        Node(
            package='sealien_mavlink_joy_bridge',
            executable='io_joy_bridge',
            name='io_joy_bridge',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'io_joy_bridge.yaml')],
        ),
    ])
