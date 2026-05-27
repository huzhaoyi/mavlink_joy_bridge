from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('sealien_mavlink_joy_bridge')
    return LaunchDescription([
        Node(
            package='sealien_mavlink_joy_bridge',
            executable='mavlink_mixed_io_receiver',
            name='mavlink_mixed_io_receiver',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'mavlink_mixed_io_receiver.yaml')],
        ),
        Node(
            package='sealien_mavlink_joy_bridge',
            executable='adc_to_joy_mapper',
            name='adc_to_joy_mapper',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'adc_to_joy_mapper.yaml')],
        ),
    ])
