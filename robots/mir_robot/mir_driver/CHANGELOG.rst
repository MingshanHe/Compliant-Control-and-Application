^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_driver
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

1.1.3 (2021-06-11)
------------------
* Merge branch 'melodic-2.8' into noetic
* Subscribe to move_base_simple/goal in relative namespace
* Use absolute topics for /tf, /tf_static, /map etc.
* Rename tf frame and topic 'odom_comb' -> 'odom'
  This is how they are called on the real MiR since MiR software 2.0.
* Fix handling of tf_static topic
  This does two things:
  1. Make the tf_static topic latched.
  2. Cache all transforms, publish as one message.
* Increase queue_size for publishers + subscribers to 10
  One case where this was a problem was the tf_static topic: Since
  multiple messages are being published at once, the subscriber often
  missed one. The tf_static topic will be fixed anyway in the next commit,
  but let's increase the queue_size anyway to avoid such bugs in the
  future.
* Update topic list to 2.8.3.1
* Reformat python code using black
* Remove outdated topics
  These topics don't exist on MiR software 2.8.3 any more (most of them
  have been removed a long time ago).
  Fixes `#37 <https://github.com/dfki-ric/mir_robot/issues/37>`_.
* Remove MirStatus
  This message was removed in MiR software 2.0 (Renamed to RobotStatus).
* Use same MirMoveBase params as real MiR (2.8.3)
  This shouldn't make a difference (it used to work before). Just removing
  one more potential source of error.
* Fix: Converts move_base_simple/goal into a move_base action. (`#62 <https://github.com/dfki-ric/mir_robot/issues/62>`_)
  At least MIR software version 2.8 does not react properly to move_base_simple/goal messages. This implements a workaround.
  Closes `#60 <https://github.com/dfki-ric/mir_robot/issues/60>`_.
* Fix: Adds subscription to "tf_static". (`#58 <https://github.com/dfki-ric/mir_robot/issues/58>`_)
  Some transformations are published on this topic and are needed to
  obtain a full tf tree. E.g. "base_footprint" to "base_link"
* Minor: Removes /particlecloud from the list of published topics. (`#57 <https://github.com/dfki-ric/mir_robot/issues/57>`_)
* Fix: Add missing dict_filter keyword argument for cmd_vel msgs (`#56 <https://github.com/dfki-ric/mir_robot/issues/56>`_)
* Remove relative_move_action (MiR => 2.4.0)
  This action was merged into the generic MirMoveBaseAction in MiR
  software 2.4.0.
* Adjust to changed MirMoveBase action (MiR >= 2.4.0)
  See `#45 <https://github.com/dfki-ric/mir_robot/issues/45>`_.
* Adjust cmd_vel topic to TwistStamped (MiR >= 2.7)
  See `#45 <https://github.com/dfki-ric/mir_robot/issues/45>`_.
* Contributors: Martin Günther, matthias-mayr

1.1.2 (2021-05-12)
------------------

1.1.1 (2021-02-11)
------------------
* Fix subscribing twice to same topic (TF etc)
  There was a flaw in the subscriber logic that caused the mir_bridge to
  subscribe multiple times to the same topic from the MiR, especially for
  latched topics. This can be seen by repeated lines in the output:
  starting to stream messages on topic 'tf'
  starting to stream messages on topic 'tf'
  starting to stream messages on topic 'tf'
  Probably related to `#64 <https://github.com/dfki-ric/mir_robot/issues/64>`_.
* Contributors: Martin Günther

1.1.0 (2020-06-30)
------------------
* Initial release into noetic
* Adapt to changes in websocket-client >= 0.49
  Ubuntu 16.04 has python-websocket  0.18
  Ubuntu 20.04 has python3-websocket 0.53
* Update scripts to Python3 (Noetic)
* Contributors: Martin Günther

1.0.6 (2020-06-30)
------------------
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.5 (2020-05-01)
------------------
* Add optional prefix parameter to fake_mir_joint_publisher (`#47 <https://github.com/dfki-ric/mir_robot/issues/47>`_)
* tf_remove_child_frames: Don't publish empty TFs
* Add sdc21x0 package, MC/currents topic
* Contributors: Martin Günther, Nils Niemann

1.0.4 (2019-05-06)
------------------
* Remove garbage file
* Contributors: Martin Günther

1.0.3 (2019-03-04)
------------------
* Make disable_map work with MiR software 2.0
  See `#5 <https://github.com/dfki-ric/mir_robot/issues/5>`_.
* mir_driver: Optionally disable the map topic + TF frame (`#6 <https://github.com/dfki-ric/mir_robot/issues/6>`_)
  This is useful when running one's own SLAM / localization nodes.
  Fixes `#5 <https://github.com/dfki-ric/mir_robot/issues/5>`_.
* Split scan_rep117 topic into two separate topics
  This fixes the problem that the back laser scanner was ignored in the
  navigation costmap in Gazebo (probably because in Gazebo, both laser
  scanners have the exact same timestamp).
* Contributors: Martin Günther

1.0.2 (2018-07-30)
------------------

1.0.1 (2018-07-17)
------------------
* mir_driver: Remove leading slashes in TF frames
* mir_driver: Install launch directory
* Contributors: Martin Günther

1.0.0 (2018-07-12)
------------------
* Initial release
* Contributors: Martin Günther
