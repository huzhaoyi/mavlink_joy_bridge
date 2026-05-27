from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os


def generate_launch_description():
    pkg = get_package_share_directory('sealien_mavlink_joy_bridge')
    prefix = get_package_prefix('sealien_mavlink_joy_bridge')
    sim_script = os.path.join(prefix, 'lib', 'sealien_mavlink_joy_bridge', 'rc_web_sim_server.py')

    return LaunchDescription([
        Node(
            package='sealien_mavlink_joy_bridge',
            executable='io_joy_bridge',
            name='io_joy_bridge',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'io_joy_bridge.yaml')],
        ),
        ExecuteProcess(
            cmd=['/usr/bin/python3', sim_script, '--config',
                 os.path.join(pkg, 'config', 'web_sim.yaml')],
            name='rc_web_sim_server',
            output='screen',
        ),
    ])
