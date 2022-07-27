^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.6 (2022-06-02)
------------------
* URDF: Downsize inertia box, move to lower back
* URDF: Pull out inertia properties
* URDF: Update masses according to data sheet
* URDF: Add mir_250
* Add arg mir_type to launch files and urdfs
* Add mir_250 meshes
* URDF: Make wheels black
* Add mir_100_v1.urdf.xacro for backwards compatibility
* Rename mir_100 -> mir
* Refactor URDF to prepare for MiR250 support
* Gazebo: Don't manually specify wheel params for diffdrive controller
* Simplify mir_100 collision mesh further
* Contributors: Martin Günther

1.1.5 (2022-02-11)
------------------
* Remove xacro comment to work around xacro bug
  Since xacro 1.14.11, xacro now also evaluates expressions in comments
  and throws an error if the substition argument is undefined. In xacro
  1.14.12, this error was changed to a warning.
  This commit removes that warning.
  Workaround for https://github.com/ros/xacro/issues/309 .
* xacro: drop --inorder option
  In-order processing became default in ROS Melodic.
* Add gazebo_plugins to dependency list (`#103 <https://github.com/dfki-ric/mir_robot/issues/103>`_)
  This is needed for the ground truth pose via p3d plugin.
* Contributors: Martin Günther, moooeeeep

1.1.4 (2021-12-10)
------------------
* Replace gazebo_plugins IMU with hector_gazebo_plugins
* Use cylinders instead of STLs for wheel collision geometries
  Fixes `#99 <https://github.com/dfki-ric/mir_robot/issues/99>`_.
* mir_debug_urdf.launch: Fix GUI display
* Contributors: Martin Günther

1.1.3 (2021-06-11)
------------------
* Merge branch 'melodic-2.8' into noetic
* Rename tf frame and topic 'odom_comb' -> 'odom'
  This is how they are called on the real MiR since MiR software 2.0.
* Contributors: Martin Günther

1.1.2 (2021-05-12)
------------------
* Fix laser scan frame_id with gazebo_plugins 2.9.2
* Contributors: Martin Günther

1.1.1 (2021-02-11)
------------------
* Add prepend_prefix_to_laser_frame to URDF and launch files
  Fixes `#65 <https://github.com/dfki-ric/mir_robot/issues/65>`_.
* Add tf_prefix to URDF and launch files
* Fix typo in robot_namespace
* Add missing 'xacro:' xml namespace prefixes
  Macro calls without 'xacro:' prefix are deprecated in Melodic and will
  be forbidden in Noetic.
* Contributors: Martin Günther

1.1.0 (2020-06-30)
------------------
* Initial release into noetic
* Contributors: Martin Günther

1.0.6 (2020-06-30)
------------------
* Update to non-deprecated robot_state_publisher node
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.5 (2020-05-01)
------------------
* Switch from Gazebo GPU laser to normal laser plugin
  The GPU laser plugin has caused multiple people problems before, because
  it is not compatible with all GPUS: `#1 <https://github.com/dfki-ric/mir_robot/issues/1>`_
  `#32 <https://github.com/dfki-ric/mir_robot/issues/32>`_
  `#46 <https://github.com/dfki-ric/mir_robot/issues/46>`_
  `#52 <https://github.com/dfki-ric/mir_robot/issues/52>`_
  The normal laser plugin directly uses the physics engine, so it doesn't
  depend on any specific GPU. Also, it doesn't slow down the simulation
  noticeably (maybe 1-2%).
* Contributors: Martin Günther

1.0.4 (2019-05-06)
------------------
* Add legacyModeNS param to gazebo_ros_control plugin
  This enables the new behavior of the plugin (pid_gains parameter are now
  in the proper namespace).
* re-added gazebo friction parameters for the wheels (`#19 <https://github.com/dfki-ric/mir_robot/issues/19>`_)
* Contributors: Martin Günther, niniemann

1.0.3 (2019-03-04)
------------------
* Merge pull request `#16 <https://github.com/dfki-ric/mir_robot/issues/16>`_ from niniemann/add-prefix-argument-to-configs
  Add prefix argument to configs
* removed prefix from plugin frameName in sick urdf
  The gazebo plugins automatically use tf_prefix, even if none is set
  (in that case it defaults to the robot namespace). That's why we can
  remove the prefix from the plugins configuration, assuming that the
  robot namespace will be equal to the prefix.
* adds $(arg prefix) to a lot of configs
  This is an important step to be able to re-parameterize move base,
  the diffdrive controller, ekf, amcl and the costmaps for adding a
  tf prefix to the robots links
* workaround eval in xacro for indigo support
* adds tf_prefix argument to imu.gazebo.urdf.xacro
* Add TFs for ultrasound sensors
* Contributors: Martin Günther, Nils Niemann

1.0.2 (2018-07-30)
------------------

1.0.1 (2018-07-17)
------------------
* gazebo: Remove leading slashes in TF frames
  TF2 doesn't like it (e.g., robot_localization).
* Contributors: Martin Günther

1.0.0 (2018-07-12)
------------------
* Initial release
* Contributors: Martin Günther
