^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.5 (2022-02-11)
------------------

1.1.4 (2021-12-10)
------------------

1.1.3 (2021-06-11)
------------------
* Merge branch 'melodic-2.8' into noetic
* Rename tf frame and topic 'odom_comb' -> 'odom'
  This is how they are called on the real MiR since MiR software 2.0.
* Reformat python code using black
* Contributors: Martin Günther

1.1.2 (2021-05-12)
------------------
* Uncomment available dependencies in noetic (`#79 <https://github.com/dfki-ric/mir_robot/issues/79>`_)
* Contributors: Oscar Lima

1.1.1 (2021-02-11)
------------------
* Add optional namespace to launch files
* Add prefix to start_planner.launch (`#67 <https://github.com/dfki-ric/mir_robot/issues/67>`_)
* Update scripts to Python3 (Noetic)
* Contributors: Martin Günther

1.1.0 (2020-06-30)
------------------
* Initial release into noetic
* Remove hector_mapping dependency (not released in noetic)
* Update scripts to Python3 (Noetic)
* Contributors: Martin Günther

1.0.6 (2020-06-30)
------------------
* Add missing matplotlib dependency
* plot_mprim: Fix color display
* Fix bug in genmprim_unicycle_highcost_5cm
  In Python3, np.arange doesn't accept floats.
* Fix some catkin_lint warnings
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.5 (2020-05-01)
------------------
* Rename hector_mapping.launch, add dependency
* genmprim.py: Improve plotting
* genmprim.py: Make executable
* SBPL: Reduce allocated_time + initial_epsilon params
  This leads to shorter planning times, but will perhaps fail on larger
  maps.
* Update mprim file to mir-software 2.0.17
  This was updated in 2.0.17 and hasn't changed through 2.6 at least.
* Add genmprim_unicycle matlab + python script, fix mprim file
* Adjust dwb params: split_path, finer trajectories (`#43 <https://github.com/dfki-ric/mir_robot/issues/43>`_)
  - use split_path option to enforce following complex paths
  - more trajectory samples over a smaller simulated time. This fixes a
  problem where the robot would stop too far away from the goal, as all
  possible trajectories either overshot the goal, or were too short to
  reach into the next gridcell of the critics.
  - remove Oscillation critic (never helped)
* added PathDistPrunedCritic for dwb (`#42 <https://github.com/dfki-ric/mir_robot/issues/42>`_)
  which works exactly like the original PathDistCritic, except that it
  searches for a local minimum in the distance from the global path to the robots
  current position. It then prunes the global_path from the start up to
  this point, therefore approximately cutting of a segment of the path
  that the robot already followed.
* Add default local_planner to move_base launch file
  This makes the hector_mapping Gazebo demo work with the instructions
  from the README (see `#32 <https://github.com/dfki-ric/mir_robot/issues/32>`_).
* Contributors: Martin Günther, Nils Niemann

1.0.4 (2019-05-06)
------------------
* Rviz config: Add planned paths + costmap from real MiR
* Contributors: Martin Günther

1.0.3 (2019-03-04)
------------------
* fix frame_id for melodic (`#18 <https://github.com/dfki-ric/mir_robot/issues/18>`_)
* Tune dwb parameters
* PathProgressCritic: Add heading score
* Use dwb_local_planner in move_base config
* Move footprint param to move_base root namespace
  This allows other move_base plugins, such as dwb_local_planner, to
  access this parameter.
* Add hector_mapping
* amcl.launch: Change default, remap service
  This is required if amcl.launch is started within a namespace.
* teb_local_planner: Fix odom topic name
* Merge pull request `#16 <https://github.com/dfki-ric/mir_robot/issues/16>`_ from niniemann/add-prefix-argument-to-configs
  Add prefix argument to configs
* adds $(arg prefix) to a lot of configs
  This is an important step to be able to re-parameterize move base,
  the diffdrive controller, ekf, amcl and the costmaps for adding a
  tf prefix to the robots links
* mir_navigation: Adjust helper node topics
* Add amcl launchfile (`#11 <https://github.com/dfki-ric/mir_robot/issues/11>`_)
  * added amcl.launch
  * changed amcl params to default mir amcl parameters
* Merge pull request `#13 <https://github.com/dfki-ric/mir_robot/issues/13>`_ from niniemann/fix-virtual-walls
  The previous configuration of the local costmap didn't work for me -- obstacles seen in the laser scans were not added, or were overridden by the virtual\_walls\_map layer. Reordering the layers and loading the virtual walls before the obstacles fixes this for me.
  Also, I added a `with_virtual_walls` parameter to `start_maps.launch` and `start_planner.launch`.
* added with_virtual_walls parameter to start_maps and start_planner
* reorder local costmap plugins
* Revert "mir_navigation: Disable virtual walls if no map file set"
  This reverts commit 0cfda301b2bb1e8b3458e698efd24a7901e5d132.
  The reason is that the `eval` keyword was introduced in kinetic, so it
  doesn't work in indigo.
* mir_navigation: Update rviz config
* mir_navigation: Disable virtual walls if no map file set
* mir_navigation: Rename virtual_walls args + files
* mir_navigation: Remove parameter first_map_only
  This parameter must be set to false (the default) when running SLAM
  (otherwise the map updates won't be received), and when running a static
  map_server it doesn't matter; even then, it should be false to allow
  restarting the map_server with a different map. Therefore this commit
  removes it altogether and leaves it at the default of "false".
* split parameter files between mapping/planning (`#10 <https://github.com/dfki-ric/mir_robot/issues/10>`_)
  The differences are simple: When mapping, first_map_only must be
  set to false, and the virtual walls plugin must not be loaded
  (else move_base will wait for a topic that is not going to be
  published).
* Document move_base params, add max_planning_retries
  Setting max_planning_retries to 10 makes the planner fail faster if the
  planning problem is infeasible. By default, there's an infinite number
  of retries, so we had to wait until the planner_patience ran out (5 s).
* Update rviz config
  Make topics relative, so that ROS_NAMESPACE=... works.
* Switch to binary sbpl_lattice_planner dependency
  ... instead of compiling from source.
* Split scan_rep117 topic into two separate topics
  This fixes the problem that the back laser scanner was ignored in the
  navigation costmap in Gazebo (probably because in Gazebo, both laser
  scanners have the exact same timestamp).
* mir_navigation: Add clear_params to move_base launch
* mir_navigation: marking + clearing were switched
  Other than misleading names, this had no effect.
* Contributors: Martin Günther, Nils Niemann, Noël Martignoni

1.0.2 (2018-07-30)
------------------

1.0.1 (2018-07-17)
------------------

1.0.0 (2018-07-12)
------------------
* Initial release
* Contributors: Martin Günther
