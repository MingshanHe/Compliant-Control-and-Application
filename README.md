# Robotic Arm Algorithms (real-devel)

  In this devel, the control algorithms can be integrated into the real robot (Universal Robot e-Series). It contains the driver and description of the real universal robots. So this devel is consist of two modules, which are separately the driver and description of the robot and the self-defined control algorithms.

## Build

```bash
$ mkdir catkin_ws/src && cd catkin_ws/src
$ git clone -b real-devel https://github.com/MingshanHe/Robotic_Arm_Algorithms.git
$ cd .. && catkin build
```

  Following the above commands, the package can be downloaded and built. It uses the `catkin build` tool, instead of the `catkin_make`. You also can use the `catkin_make` to build this package.

## Usage

### 1. Connect to the Robot

```bash
$ roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.101 kinematics_config:=$(rospack find ur_calibration)/etc/my_robot_calibration.yaml
```

  In this command, I have appointed the UR5e Robot and its IP address. **You need to correct it in your situation including the calibration file**. In launch file, I have used my controller: `cartesian_velocity_controller`, so it will be different from the official document.

### 2. Run the Control Algorithm

  After connect to the real robot, it need to run the control algorithm based on the driver. In this package, I have provide some control algorithms to test, like admittance control and others. If you want to run the **admittance control algorithm**, you can run the launch file as follow,

```bash
$ roslaunch Admittance Admittance.launch
```

