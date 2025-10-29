from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('safe_scout_simulator')
    config_file = os.path.join(pkg_dir, 'config', 'ground_truth_params.yaml')

    ground_truth_server = Node(
        package='safe_scout_simulator',
        executable='ground_truth_server',
        name='ground_truth_server',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )

    spatial_measurement_publisher = Node(
        package='safe_scout_simulator',
        executable='spatial_measurement_publisher.py',
        name='spatial_measurement_publisher',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )

    return LaunchDescription([
        ground_truth_server,
        spatial_measurement_publisher
    ])
