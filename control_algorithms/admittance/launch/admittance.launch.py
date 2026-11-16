"""Admittance controller for one of the robots in robots/.

    ros2 launch admittance admittance.launch.py robot:=ur5
    ros2 launch admittance admittance.launch.py robot:=franka topic_wrench_state:=/my/wrench

Parameters come from config/<robot>.yaml (or `config:=/path/to/file.yaml`);
the topics can be overridden from the command line.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    config = arg('config') or os.path.join(
        get_package_share_directory('admittance'), 'config', f"{arg('robot')}.yaml")
    return [Node(
        package='admittance', executable='admittance_node', name='admittance', output='screen',
        parameters=[config, {
            'topic_arm_state': arg('topic_arm_state'),
            'topic_arm_command': arg('topic_arm_command'),
            'topic_wrench_state': arg('topic_wrench_state'),
            'use_sim_time': arg('use_sim_time') == 'true',
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='ur5',
                              description='Robot config to load: ur5, franka or b601_dm'),
        DeclareLaunchArgument('config', default_value='',
                              description='Parameter file (default: config/<robot>.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('topic_arm_state',
                              default_value='/cartesian_velocity_controller/ee_state'),
        DeclareLaunchArgument('topic_arm_command',
                              default_value='/cartesian_velocity_controller/command_cart_vel'),
        DeclareLaunchArgument('topic_wrench_state',
                              default_value='/force_torque_sensor_broadcaster/wrench'),
        OpaqueFunction(function=_nodes),
    ])
