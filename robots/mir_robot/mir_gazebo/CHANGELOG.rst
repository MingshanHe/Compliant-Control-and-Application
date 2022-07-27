^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.6 (2022-06-02)
------------------
* Add arg mir_type to launch files and urdfs
* Rename mir_100 -> mir
  This is in preparation of mir_250 support.
* Contributors: Martin Günther

1.1.5 (2022-02-11)
------------------

1.1.4 (2021-12-10)
------------------
* Remove outdated comment
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
* mir_gazebo: Add model_name arg
* Move joint_state_publisher to mir_gazebo_common.launch
* Add optional namespace to launch files
* Add prepend_prefix_to_laser_frame to URDF and launch files
  Fixes `#65 <https://github.com/dfki-ric/mir_robot/issues/65>`_.
* Add tf_prefix to URDF and launch files
* Contributors: Martin Günther

1.1.0 (2020-06-30)
------------------
* Initial release into noetic
* Contributors: Martin Günther

1.0.6 (2020-06-30)
------------------
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.5 (2020-05-01)
------------------

1.0.4 (2019-05-06)
------------------
* Fix gazebo launch file
  Before this commit, the mobile base plugin couldn't initialize, because
  subst_value didn't work.
* Contributors: Martin Günther

1.0.3 (2019-03-04)
------------------
* Add hector_mapping
* fake_localization.launch: Add frame id args
* Merge pull request `#16 <https://github.com/dfki-ric/mir_robot/issues/16>`_ from niniemann/add-prefix-argument-to-configs
  Add prefix argument to configs
* adds $(arg prefix) to a lot of configs
  This is an important step to be able to re-parameterize move base,
  the diffdrive controller, ekf, amcl and the costmaps for adding a
  tf prefix to the robots links
* Fix translation error in odom_comb (`#12 <https://github.com/dfki-ric/mir_robot/issues/12>`_)
  Previously, the ekf localization only computed a correct orientation, but the translation still followed the pure odometry data. This led to strange errors where the robot would move sideways (despite only having a diff drive).
  This PR changes the ekf configuration to not use any position information from the odometry, but to integrate the velocities, which fixes this problem.
* Split scan_rep117 topic into two separate topics
  This fixes the problem that the back laser scanner was ignored in the
  navigation costmap in Gazebo (probably because in Gazebo, both laser
  scanners have the exact same timestamp).
* Contributors: Martin Günther, Nils Niemann

1.0.2 (2018-07-30)
------------------
* mir_gazebo: Install config directory
* Contributors: Martin Günther

1.0.1 (2018-07-17)
------------------
* gazebo: Replace robot_pose_ekf with robot_localization
  robot_pose_ekf is deprecated, and has been removed from the navigation
  stack starting in melodic.
* gazebo: Adjust ekf.yaml
* gazebo: Copy robot_localization/ekf_template.yaml
  ... for modification.
* Contributors: Martin Günther

1.0.0 (2018-07-12)
------------------
* Initial release
* Contributors: Martin Günther
