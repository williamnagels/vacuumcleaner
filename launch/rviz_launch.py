"""Launch the cpp_code executable in this package"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import launch_ros.actions
import os
from launch_ros.actions import Node
def generate_launch_description():

    bringup_dir = get_package_share_directory('vacuumcleaner')
    launch_dir = os.path.join(bringup_dir, 'launch')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'model')
    rviz_config_file = os.path.join(bringup_dir, "config.rviz")
    print(rviz_config_file)
    use_sim_time = 'true'
    return LaunchDescription([
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_file],
            ),
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf_file]),
        Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            node_name='joint_state_publisher',
            output='screen',
            parameters=[{'dependent_joints': '[]', 'use_gui': 'true'}],
            arguments=[urdf_file])
   ])
