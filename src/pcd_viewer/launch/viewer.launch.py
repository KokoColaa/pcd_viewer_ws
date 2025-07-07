from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pcd_viewer')
    pcd_file = os.path.join(pkg_share, 'rmul_2024.pcd')
    config_file = os.path.join(pkg_share, 'config', 'transform.yaml')
    return LaunchDescription([
        Node(
            package='pcd_viewer',
            executable='pcd_viewer_node',
            name='pcd_viewer',
            output='screen',
            parameters=[
                {'pcd_file': pcd_file},
                {'frame_id': 'map'},
                {'child_frame_id': 'pcd_frame'},
                {'config_file': config_file},
                {'topic_name': 'pcd_points'}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'viewer.rviz')],
            output='screen'
        )
    ])