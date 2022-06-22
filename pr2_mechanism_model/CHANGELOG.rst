^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_mechanism_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.21 (2022-06-22)
-------------------
* set catkin_add_gtest within CATKIN_ENABLE_TESTING (`#346 <https://github.com/pr2/pr2_mechanism/issues/346>`_)
  fix with CATKIN_ENABLE_TESTING=OFF
* Parse old & new transmission style (`#345 <https://github.com/pr2/pr2_mechanism/issues/345>`_)
  The transmission style changed in ros_control, to make them compatible
  we adjusted them in the pr2_description
  This came up when we integrated the shadow-hand- and the PR2-model
* Contributors: Kei Okada, Michael Görner

1.8.20 (2022-04-11)
-------------------

1.8.19 (2022-04-10)
-------------------

1.8.18 (2018-09-11)
-------------------
* Merge pull request `#338 <https://github.com/pr2/pr2_mechanism/issues/338>`_ from k-okada/add_travis
  update travis.yml
* fix urdf::JointConstSharedPtr for test directory
* fix for urdfmodel >= 1.0.0 (melodic)
* to pass catkin run_tests, partially copied from https://github.com/PR2/pr2_mechanism/pull/329
* Contributors: Kei Okada

1.8.17 (2018-02-13)
-------------------
* Merge pull request `#336 <https://github.com/pr2/pr2_mechanism/issues/336>`_ from k-okada/hardware_interface_0.13.0
  set hardware_interface >= 0.13.0
* set hardware_interface >= 0.13.0
  https://github.com/ros-controls/ros_control/pull/285 (which is included in hardware_interface 0.13.0) allow us to use class `hardware_interface::HardwareResourceManager` API
* Merge pull request `#335 <https://github.com/pr2/pr2_mechanism/issues/335>`_ from k-okada/fix_cmake
  fix compile warning
* fix compile warning
* Merge pull request `#332 <https://github.com/pr2/pr2_mechanism/issues/332>`_ from k-okada/maintain
  change maintainer to ROS orphaned package maintainer
* change maintainer to ROS orphaned package maintainer
* Contributors: Kei Okada

1.8.15 (2015-01-13)
-------------------

1.8.14 (2015-01-13)
-------------------

1.8.13 (2014-12-16)
-------------------
* Updated to use cmake_modules
* Updated to use cmake_modules
* Contributors: Jack Thompson
