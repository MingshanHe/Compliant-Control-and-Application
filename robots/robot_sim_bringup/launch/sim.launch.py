"""Start one of the robots in robots/ in Gazebo Sim with ros2_control.

    ros2 launch robot_sim_bringup sim.launch.py robot:=ur5
    ros2 launch robot_sim_bringup sim.launch.py robot:=franka world:=polish
    ros2 launch robot_sim_bringup sim.launch.py robot:=b601_dm gui:=false rviz:=true

Every robot package `<robot>_sim` provides config/sim.yaml (what to load),
urdf/<robot>.urdf.xacro and config/controllers.yaml. After start-up these
controllers are active:

    joint_state_broadcaster          /joint_states
    force_torque_sensor_broadcaster  /force_torque_sensor_broadcaster/wrench
    cartesian_velocity_controller    /cartesian_velocity_controller/command_cart_vel  (in)
                                     /cartesian_velocity_controller/ee_state          (out)
"""
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import yaml

ROBOTS = ('ur5', 'franka', 'b601_dm')  # any <robot>_sim package following robots/README.md works
WORLDS = ('empty', 'polish')


def _launch_setup(context):
    def arg(name):
        return context.launch_configurations[name]

    robot = arg('robot')
    sim_pkg = f'{robot}_sim'
    try:
        sim_share = get_package_share_directory(sim_pkg)
    except PackageNotFoundError:
        raise RuntimeError(f"No package '{sim_pkg}' for robot '{robot}' (available: {ROBOTS})")
    bringup_share = get_package_share_directory('robot_sim_bringup')
    with open(os.path.join(sim_share, 'config', 'sim.yaml'), 'r') as f:
        cfg = yaml.safe_load(f)['robot']
    for pkg in cfg.get('resource_packages', []):
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError:
            raise RuntimeError(
                f"Package '{pkg}' (needed by {sim_pkg}) was not found. Packages that are not "
                'released for this ROS distro are listed in robots/robots.repos: run '
                "'vcs import robots < robots/robots.repos', rebuild and source the workspace.")

    # Robot description
    controllers_file = os.path.join(sim_share, cfg.get('controllers_file',
                                                       'config/controllers.yaml'))
    robot_description = xacro.process_file(
        os.path.join(sim_share, cfg['description_file']),
        mappings={'controllers_file': controllers_file},
    ).toxml()

    # World
    world = arg('world')
    if world in WORLDS:
        world_file = os.path.join(bringup_share, 'worlds', 'empty.sdf')
    elif os.path.isfile(world):
        world_file = world
    else:
        raise RuntimeError(f"Unknown world '{world}', choose one of {WORLDS} or give a .sdf path")

    # Gazebo has to find the meshes of every description package (package:// URIs
    # become model:// URIs, resolved against GZ_SIM_RESOURCE_PATH).
    resource_dirs = {os.path.dirname(get_package_share_directory(p))
                     for p in cfg.get('resource_packages', []) + [sim_pkg]}
    resource_dirs.add(os.path.join(bringup_share, 'models'))
    actions = [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', d) for d in sorted(resource_dirs)]

    ros_gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')
    if arg('gui') == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_launch, 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f"-r -v {arg('gz_verbosity')} {world_file}",
                              'on_exit_shutdown': 'true'}.items()))
    else:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_launch, 'gz_server.launch.py')),
            launch_arguments={'world_sdf_file': world_file}.items()))

    use_sim_time = {'use_sim_time': True}

    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'))

    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}, use_sim_time]))

    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', robot, '-allow_renaming', 'true'])
    actions.append(spawn_robot)

    controllers = ['joint_state_broadcaster', 'force_torque_sensor_broadcaster']
    if arg('controller'):
        controllers.append(arg('controller'))
    spawn_controllers = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=controllers + ['--param-file', controllers_file,
                                 '--controller-manager-timeout', '120'],
        parameters=[use_sim_time])
    actions.append(RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[spawn_controllers])))

    if world == 'polish':
        x, y, z, roll, pitch, yaw = (str(v) for v in cfg['workpiece_pose'])
        actions.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-file', os.path.join(bringup_share, 'models', 'car_wheel', 'model.sdf'),
                       '-name', 'car_wheel', '-x', x, '-y', y, '-z', z,
                       '-R', roll, '-P', pitch, '-Y', yaw]))

    if arg('rviz') == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'sim.rviz'),
                       '-f', cfg.get('fixed_frame', 'world')],
            parameters=[use_sim_time]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='ur5',
                              description=f"Robot to simulate: {', '.join(ROBOTS)}"),
        DeclareLaunchArgument('world', default_value='empty',
                              description="'empty', 'polish' (adds the car-wheel work piece) "
                                          'or a path to an .sdf world'),
        DeclareLaunchArgument('controller', default_value='cartesian_velocity_controller',
                              description="Arm controller to start ('' for none)"),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Start the Gazebo GUI (false: headless server)'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gz_verbosity', default_value='1'),
        OpaqueFunction(function=_launch_setup),
    ])
