from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory('wall_follow')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follow',
            executable='wall_follow',
            name='wall_follow',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'lidar'],
            name='my_static_tf_pub'
        ),
    ])