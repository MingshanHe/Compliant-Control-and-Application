"""Admittance controller driven by a synthetic wrench (no contact needed).

    ros2 launch admittance admittance_test.launch.py robot:=ur5
    ros2 launch admittance admittance_test.launch.py robot:=franka amplitude:=2.0 direction:="1 0 0"

wrench_signal_generate publishes a +/- amplitude square wave (10 s period)
along `direction` in the robot base frame; the end effector moves back and
forth around its desired pose. The default amplitude/direction for each robot
are in the `wrench_signal_generate` section of config/<robot>.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _nodes(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    use_sim_time = arg('use_sim_time') == 'true'
    config = arg('config') or os.path.join(
        get_package_share_directory('admittance'), 'config', f"{arg('robot')}.yaml")
    with open(config, 'r') as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']

    topic = '/wrench_signal'
    generator = {'topic': topic, 'frame_id': params['base_link'], 'use_sim_time': use_sim_time}
    if arg('amplitude'):
        generator['amplitude'] = float(arg('amplitude'))
    if arg('direction'):
        generator['direction'] = [float(v) for v in arg('direction').replace(',', ' ').split()]

    return [
        Node(
            package='admittance', executable='admittance_node', name='admittance',
            output='screen',
            parameters=[config, {'topic_wrench_state': topic, 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='admittance', executable='wrench_signal_generate',
            name='wrench_signal_generate', output='screen',
            parameters=[config, generator],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='ur5',
                              description='Robot config to load: ur5, franka or b601_dm'),
        DeclareLaunchArgument('config', default_value='',
                              description='Parameter file (default: config/<robot>.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('amplitude', default_value='',
                              description='Force of the square wave [N] (default from config)'),
        DeclareLaunchArgument('direction', default_value='',
                              description='Force direction in the base frame, e.g. "0 0 1"'),
        OpaqueFunction(function=_nodes),
    ])
