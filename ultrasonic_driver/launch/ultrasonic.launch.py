from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    param_dir = os.path.join(get_package_share_directory('ultrasonic_driver'), 'params', 'config.yaml')

    return LaunchDescription([
        Node(
            package='ultrasonic_driver',
            namespace='ultrasonic',
            executable='ultrasonic_driver_node',
            name='ultrasonic_driver_node',
            parameters=[param_dir],
            remappings=[
                ('/distance', '/distance'),
                ('/relative_velocity', '/relative_velocity')
            ]
        ),
    ])