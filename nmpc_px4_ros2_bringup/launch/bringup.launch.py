from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmpc_px4_ros2_utils',
            executable='odom_repub_node',
            name='odom_repub_node'
        ),
        Node(
            package='nmpc_px4_ros2_utils',
            executable='ref_traj_pub_node',
            name='ref_traj_pub_node'
        ),
        Node(
            package='nmpc_px4_ros2',
            executable='nmpc_flight_mode',
            name='nmpc_flight_mode'
        )
    ])