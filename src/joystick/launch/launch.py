from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory('joystick')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            parameters=[{'sticky_buttons': True}]
        ),
        Node(
            package='joystick',
            executable='map_joy_to_ack',
            name='map',
            output='screen'
        )
    ])