from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lift_controller')
    install_config = os.path.join(pkg_share, 'config', 'lift_config.yaml')

    # 从 install 路径推导源码路径,优先加载源码目录的配置文件(修改后无需 colcon build)
    # pkg_share = <workspace>/install/lift_controller/share/lift_controller
    ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    source_config = os.path.join(ws_dir, 'src', 'lift_controller', 'config', 'lift_config.yaml')

    # 优先使用源码目录的配置,找不到时回退到 install 目录
    params_file = source_config if os.path.exists(source_config) else install_config
    print(f'[lift_controller] Loading config: {params_file}')

    return LaunchDescription([
        Node(
            package='lift_controller',
            executable='lift_controller_node',
            name='lift_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])
