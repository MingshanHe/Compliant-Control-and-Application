"""Hybrid position/force control for one of the robots in robots/.

    ros2 launch robot_sim_bringup sim.launch.py robot:=ur5 world:=polish
    ros2 launch hybrid_position_force_control hybrid_position_force_control.launch.py robot:=ur5

Then send Cartesian position targets (base frame) to the pose command topic, e.g.

    ros2 topic pub --once /desired_cartesian_pose_cmd geometry_msgs/msg/Pose \
      "{position: {x: 0.5, y: 0.1, z: 0.3}, orientation: {w: 1.0}}"
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
        get_package_share_directory('hybrid_position_force_control'), 'config',
        f"{arg('robot')}.yaml")
    return [Node(
        package='hybrid_position_force_control', executable='hybrid_position_force_control_node',
        name='hybrid_position_force_control', output='screen',
        parameters=[config, {
            'topic_arm_state': arg('topic_arm_state'),
            'topic_arm_command': arg('topic_arm_command'),
            'topic_wrench_state': arg('topic_wrench_state'),
            'topic_cart_pose_cmd': arg('topic_cart_pose_cmd'),
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
        DeclareLaunchArgument('topic_cart_pose_cmd', default_value='/desired_cartesian_pose_cmd'),
        OpaqueFunction(function=_nodes),
    ])
