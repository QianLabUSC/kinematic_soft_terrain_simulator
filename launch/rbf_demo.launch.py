from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('safe_scout_simulator')

    # Path to the configuration file
    config_file = os.path.join(pkg_dir, 'config', 'ground_truth_params.yaml')

    # Ground truth server node
    ground_truth_server = Node(
        package='safe_scout_simulator',
        executable='ground_truth_server',
        name='ground_truth_server',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )

    # RBF visualization node (delayed start to ensure server is ready)
    rbf_visualization = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='safe_scout_simulator',
                executable='rbf_visualization.py',
                name='rbf_visualization',
                output='screen',
                parameters=[{
                    'num_training_points': 200,
                    'grid_resolution': 50,
                    'area_min_x': -15.0,
                    'area_max_x': 15.0,
                    'area_min_y': -15.0,
                    'area_max_y': 15.0,
                }],
                emulate_tty=True
            )
        ]
    )

    return LaunchDescription([
        ground_truth_server,
        rbf_visualization
    ])
