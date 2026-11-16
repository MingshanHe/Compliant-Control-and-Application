# robots

All robot models used by the compliant controllers, organised so that every
robot looks the same to the rest of the repository.

```
robots/
├── robots.repos                   # upstream sources that are not on apt (vcs import)
├── robot_sim_bringup/             # shared: Gazebo world, work piece, F/T-probe xacro, sim.launch.py, RViz
├── ur5/
│   └── ur5_sim/                   # UR5     (ur_description from apt)
├── franka/
│   ├── franka_description/        # FR3 model, pulled by robots.repos (git-ignored)
│   └── franka_sim/                # Franka Research 3
└── b601_dm/
    ├── b601_dm_description/       # Seeed reBot Arm B601-DM model + meshes (vendored, Apache-2.0)
    └── b601_dm_sim/               # B601-DM
```

| robot     | `robot:=` | description source                    | base frame  | controlled frame | DoF |
|-----------|-----------|----------------------------------------|-------------|------------------|-----|
| UR5       | `ur5`     | `ur_description` (apt, `ur_type=ur5`)  | `base_link` | `tcp`            | 6   |
| Franka FR3| `franka`  | `franka_description` 2.9.0 (source)    | `fr3_link0` | `tcp`            | 7   |
| B601-DM   | `b601_dm` | `b601_dm_description` (vendored)       | `base_link` | `tcp`            | 6 (+ gripper) |

```bash
ros2 launch robot_sim_bringup sim.launch.py robot:=ur5      # or franka / b601_dm
ros2 launch robot_sim_bringup sim.launch.py robot:=franka world:=polish rviz:=true
```

Launch arguments: `robot`, `world` (`empty`, `polish` = adds the car-wheel work
piece, or a path to an `.sdf`), `gui` (`false` runs a headless server),
`rviz`, `controller` (`''` starts only the broadcasters), `gz_verbosity`.

## What every robot provides

Each `<robot>_sim` package follows the same contract, which is all that
`sim.launch.py` and the control algorithms rely on:

| file                          | content |
|-------------------------------|---------|
| `config/sim.yaml`             | xacro file, controller file, packages whose meshes Gazebo must find, work-piece pose |
| `urdf/<robot>.urdf.xacro`     | arm fixed to `world` + `ft_probe_tool` (F/T sensor + spring-loaded probe, frame `tcp`) + `<ros2_control>` with `gz_ros2_control/GazeboSimSystem` |
| `config/controllers.yaml`     | `joint_state_broadcaster`, `force_torque_sensor_broadcaster`, `cartesian_velocity_controller` |

so that, for every robot, the simulation exposes the same interface:

| topic | type | |
|-------|------|-|
| `/cartesian_velocity_controller/command_cart_vel` | `geometry_msgs/msg/Twist` | end-effector twist command (base frame) |
| `/cartesian_velocity_controller/ee_state` | `cartesian_state_msgs/msg/PoseTwist` | `tcp` pose and twist (base frame) |
| `/force_torque_sensor_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | wrench at the wrist sensor, `frame_id: ft_sensor_link` |
| `/joint_states` | `sensor_msgs/msg/JointState` | |

The F/T sensor reports the wrench that the tool applies to the flange
(contact wrench + 0.78 N tool weight). The probe head is a passive prismatic
joint with a 2000 N/m spring: it keeps contact with rigid work pieces stable
for a velocity-controlled arm, like the compliant heads of real polishing tools.

Home poses (initial joint positions in the xacro, tool pointing down):

| robot   | home joints | `tcp` in base frame |
|---------|-------------|---------------------|
| UR5     | `[0, -π/2, π/2, -π/2, -π/2, 0]` | `[0.487, 0.109, 0.290]` |
| FR3     | `[0, -π/4, 0, -3π/4, 0, π/2, π/4]` (ready pose) | `[0.307, 0.000, 0.448]` |
| B601-DM | `[0, -1.5, -1.25, 1.05, 0, 0]`, on a 0.40 m pedestal | `[0.341, 0.000, 0.056]` (tool ≈15° from vertical; joint4 limits the pitch) |

## Getting the sources

`ur_description` is installed with apt. `franka_description` is not released for
ROS 2 Lyrical, so it is cloned at a fixed tag:

```bash
vcs import robots < robots/robots.repos
```

The B601-DM model is vendored in `b601_dm/b601_dm_description` (Apache-2.0,
see its `NOTICE` for the upstream commit and the changes made).

## Adding a robot

1. Make the model available: an apt package, an entry in `robots.repos`, or a
   vendored `<robot>_description` package.
2. Create `robots/<robot>/<robot>_sim` (copy `ur5/ur5_sim`):
   - `urdf/<robot>.urdf.xacro`: include the arm, `world` link,
     `xacro:ft_probe_tool` on the flange, a `<ros2_control>` block built from
     `xacro:sim_joint`, `xacro:sim_probe_joint` and `xacro:sim_ft_sensor`, and
     `xacro:gz_ros2_control_plugin` (all in `robot_sim_bringup/urdf/sim_common.xacro`).
   - `config/controllers.yaml`: root/tip links and joints of the arm.
   - `config/sim.yaml`: files, `resource_packages`, `workpiece_pose`.
3. Add `config/<robot>.yaml` to `control_algorithms/admittance` and
   `control_algorithms/hybrid_position_force_control` (base link, desired pose =
   home `tcp` pose, gains).
4. `ros2 launch robot_sim_bringup sim.launch.py robot:=<robot>`
