import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # TODO: Set LD_LIBRARY_PATH env var with paths to acados lib and model lib until its possible to do it directly
    ref_traj_config = os.path.join(get_package_share_directory('nmpc_px4_ros2_bringup'), 'config', 'ref_traj_params.yaml')
    nmpc_config = os.path.join(get_package_share_directory('nmpc_px4_ros2_bringup'), 'config', 'nmpc_params.yaml')

    return LaunchDescription([
        Node(
            package='nmpc_px4_ros2_utils',
            executable='odom_repub_node',
            name='odom_repub_node'
        ),
        Node(
            package='nmpc_px4_ros2_utils',
            executable='ref_traj_pub_node',
            name='ref_traj_pub_node',
            parameters=[ref_traj_config],
        ),
        Node(
            package='nmpc_px4_ros2',
            executable='nmpc_flight_mode',
            name='nmpc_flight_mode',
            parameters=[nmpc_config],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('nmpc_px4_ros2_bringup'), 'rviz', 'config.rviz')],
        )
    ])