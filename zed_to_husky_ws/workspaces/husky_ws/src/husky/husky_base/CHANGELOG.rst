^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2022-05-19)
------------------

1.0.7 (2022-05-19)
------------------
* Renamed all launch files to *.launch.py.
* Contributors: Tony Baltovski

1.0.6 (2022-05-18)
------------------
* Added searching for left and right joints rather than assuming order.
* Contributors: Tony Baltovski

1.0.5 (2022-05-05)
------------------
* Split teleop launch into two files since simulation doesn't need actual joystick and will spam warmings.
* [husky_base] Fixed spawner.py bug.
* [husky_base] Added missing includes for base_launch.py.
* Revamped tele-op launch.
* Fix chnaged dependency in rolling.
* Contributors: Denis Štogl, Tony Baltovski

1.0.4 (2022-03-15)
------------------
* Merge pull request `#191 <https://github.com/husky/husky/issues/191>`_ from StoglRobotics-forks/gazebo-sim-integration-fixes
  Gazebo sim integration fixes
* Contributors: Tony Baltovski

1.0.3 (2021-11-30)
------------------
* [husky_base] Switched Boost static assert to C++ standard library.
* Contributors: Tony Baltovski

1.0.2 (2021-11-16)
------------------
* Removed unused parameters.
* [husky_base] Removed duplicate node in launch.
* Contributors: Tony Baltovski

1.0.1 (2021-11-12)
------------------

1.0.0 (2021-11-07)
------------------
* Initial Gazebo Classic changes.
* Updates to use ros2_control.
* [husky_base] Fixed comparison warnings.
* [husky_base] Fixed logging variable order.
* [husky_base] Populated hardware states and removed joints struct.
* Initial attempt at ros2_control.
* [husky_base] Updated horizon_legacy library for C++11.
* [husky_base] Linting changes to horizon_legacy library.
* Contributors: Tony Baltovski

0.4.4 (2020-08-13)
------------------
* Removed Paul Bovbel as maintainer.
* Contributors: Tony Baltovski

0.4.3 (2020-04-20)
------------------

0.4.2 (2019-12-11)
------------------

0.4.1 (2019-09-30)
------------------

0.4.0 (2019-08-01)
------------------

0.3.4 (2019-08-01)
------------------

0.3.3 (2019-04-18)
------------------

0.3.2 (2019-03-25)
------------------

0.3.1 (2018-08-02)
------------------

0.3.0 (2018-04-11)
------------------
* Fix default tyre radius
* changed the tire radius in base.launch to reflect a 13 inch Husky outdoor tire
* Remove defunct email address
* Updated maintainers.
* Update bringup for multirobot
* Update URDF for multirobot
* Move packages into monorepo for kinetic; strip out ur packages
* Contributors: Martin Cote, Paul Bovbel, Tony Baltovski, Wolfgang Merkt

0.2.6 (2016-10-03)
------------------
* Adding support for the UM7 IMU.
* Contributors: Tony Baltovski

0.2.5 (2015-12-31)
------------------
* Fix absolute value to handle negative rollover readings effectively
* Another bitwise fix, now for x86.
* Formatting
* Fix length complement check.
  There's a subtle difference in how ~ is implemented in aarch64 which
  causes this check to fail. The new implementation should work on x86
  and ARM.
* Contributors: Mike Purvis, Paul Bovbel

0.2.4 (2015-07-08)
------------------

0.2.3 (2015-04-08)
------------------
* Integrate husky_customization workflow
* Contributors: Paul Bovbel

0.2.2 (2015-03-23)
------------------
* Fix package urls
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------
* Add missing dependencies
* Contributors: Paul Bovbel

0.2.0 (2015-03-23)
------------------
* Add UR5_ENABLED envvar
* Contributors: Paul Bovbel

0.1.5 (2015-02-19)
------------------
* Fix duration cast
* Contributors: Paul Bovbel

0.1.4 (2015-02-13)
------------------
* Correct issues with ROS time discontinuities - now using monotonic time source
* Implement a sane retry policy for communication with MCU
* Contributors: Paul Bovbel

0.1.3 (2015-01-30)
------------------
* Update description and maintainers
* Contributors: Paul Bovbel

0.1.2 (2015-01-20)
------------------
* Fix library install location
* Contributors: Paul Bovbel

0.1.1 (2015-01-13)
------------------
* Add missing description dependency
* Contributors: Paul Bovbel

0.1.0 (2015-01-12)
------------------
* Fixed encoder overflow issue
* Ported to ros_control for Indigo release
* Contributors: Mike Purvis, Paul Bovbel, finostro

0.0.5 (2013-10-04)
------------------
* Mark the config directory to install.

0.0.4 (2013-10-03)
------------------
* Parameterize husky port in env variable.

0.0.3 (2013-09-24)
------------------
* Add launchfile check.
* removing imu processing by dead_reckoning.py
* removing dynamic reconfigure from dead_reckoning because it was only there for handling gyro correction
* adding diagnostic aggregator and its related config file under config/diag_agg.yaml

0.0.2 (2013-09-11)
------------------
* Fix diagnostic_msgs dependency.

0.0.1 (2013-09-11)
------------------
* New husky_base package for Hydro, which contains the nodes
  formerly in husky_bringup.
