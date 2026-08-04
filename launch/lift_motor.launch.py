from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lift_controller')

    # The YAML file uses the node name "lift_controller" as the parameter
    # namespace, so the node name must match for parameters to load.
    params_file = os.path.join(pkg_share, 'config', 'lift_config.yaml')

    return LaunchDescription([
        Node(
            package='lift_controller',
            executable='lift_controller_node',
            name='lift_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])
