"""Show the B601-DM model in RViz with joint sliders.

    ros2 launch b601_dm_description view_b601_dm.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = get_package_share_directory('b601_dm_description')
    description = xacro.process_file(os.path.join(share, 'urdf', 'b601_dm.urdf.xacro')).toxml()
    rviz = os.path.join(get_package_share_directory('robot_sim_bringup'), 'rviz', 'sim.rviz')
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz, '-f', 'world']),
    ])
