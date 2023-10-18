from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory('lidar')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar',
            executable='lidar',
            name='lidar',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'laser'],
            name='my_static_tf_pub'
        ),
    ])