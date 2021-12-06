# Robotic_Arm_Algorithms

  It contains the common algorithms in robotic arm, and will be recording the development as soon as I have completed the any one algorithm. The chosen robot is **universal robot** which is the most popular collaborate robot in the world.

## Algorithm List

### 1. Admittance

  In compliant control, admittance control and impedance control are the most common control algorithms and equivalent each other. For running this algorithms, there are only two commands:

  First of all, running a gazebo environment and upload a universal robot. I usually chose a **ur5e**.

```bash
$ roslaunch ur_e_gazebo ur5e.launch
```

  Then, running the admittance control algorithm and you can observe the performance of it.

```bash
$ roslaunch Admittance Admittance_test.lanch
```

  In this lanch file, it contains a node to publish a wrench signal to imitate an external force caused by operator. If you want to run it without publish node and to publish by yourself other package, you can run the following command.

```bash
$ roslaunch Admittance Admittance.launch
```

![](Image/Admittance.gif)
