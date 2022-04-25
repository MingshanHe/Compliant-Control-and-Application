^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_dwb_critics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.5 (2022-02-11)
------------------

1.1.4 (2021-12-10)
------------------

1.1.3 (2021-06-11)
------------------
* Merge branch 'melodic-2.8' into noetic
* Reformat python code using black
* Contributors: Martin Günther

1.1.2 (2021-05-12)
------------------

1.1.1 (2021-02-11)
------------------
* Fix bug in path_dist_pruned
  With some paths, the previous code crashed with "terminate called after throwing an instance
  of 'std::bad_alloc'".
* Contributors: Martin Günther

1.1.0 (2020-06-30)
------------------
* Initial release into noetic
* Update scripts to Python3 (Noetic)
* Contributors: Martin Günther

1.0.6 (2020-06-30)
------------------
* Add missing matplotlib dependency
* Fix some catkin_lint warnings
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.5 (2020-05-01)
------------------
* mir_dwb_critics: Add plot_dwb_scores.py
* mir_dwb_critics: Improve print_dwb_scores output
* added PathDistPrunedCritic for dwb (`#42 <https://github.com/dfki-ric/mir_robot/issues/42>`_)
  which works exactly like the original PathDistCritic, except that it
  searches for a local minimum in the distance from the global path to the robots
  current position. It then prunes the global_path from the start up to
  this point, therefore approximately cutting of a segment of the path
  that the robot already followed.
* Contributors: Martin Günther, Nils Niemann

1.0.4 (2019-05-06)
------------------

1.0.3 (2019-03-04)
------------------
* PathProgressCritic: Add heading score
* Add package: mir_dwb_critics
* Contributors: Martin Günther

1.0.2 (2018-07-30)
------------------

1.0.1 (2018-07-17)
------------------

1.0.0 (2018-07-12)
------------------
