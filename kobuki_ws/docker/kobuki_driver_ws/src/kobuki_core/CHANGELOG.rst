=========
Changelog
=========

Forthcoming
-----------
* ...

1.4.0 (2022-10-24)
------------------
* [demos] log levels demo added, `#42 <https://github.com/kobuki-base/kobuki_core/issues/42>`_
* [driver] bugfix major, minor, patch macro conflicts, `#44 <https://github.com/kobuki-base/kobuki_core/issues/44>`_
* [demos] style cleanups, `#47 <https://github.com/kobuki-base/kobuki_core/pull/47>`_
* [driver] grammar fixes, `#47 <https://github.com/kobuki-base/kobuki_core/pull/47>`_

1.3.1 (2020-09-15)
------------------
* [keyop] protect command variables with a mutex, `#41 <https://github.com/kobuki-base/kobuki_core/issues/41>`_
* [core] ros_ prefixes dropped, custom_logging & raw_data_stream demos, `#40 <https://github.com/kobuki-base/kobuki_core/issues/40>`_
* [infra] recommended firmware versions 1.1.4, 1.2.0, `#37 <https://github.com/kobuki-base/kobuki_core/issues/37>`_

1.3.0 (2020-09-12)
------------------
* [all] LegacyPose2D -> Vector3d, `#36 <https://github.com/kobuki-base/kobuki_core/issues/36>`_

1.2.0 (2020-08-30)
------------------
* [infra] kobuki_dock_drive and kobuki_driver merged into kobuki_core, `#31 <https://github.com/kobuki-base/kobuki_core/issues/31>`_
* [tools] kobuki_simple_keyop -> kobuki-simple-keyop, `#35 <https://github.com/kobuki-base/kobuki_core/issues/35>`_.
* [tools] kobuki_version_info -> kobuki-version-info, `#35 <https://github.com/kobuki-base/kobuki_core/issues/35>`_.

1.1.1 (2020-08-19)
------------------
* [udev] bugfix low latency ftdi operation, `#28 <https://github.com/kobuki-base/kobuki_core/pull/28>`_.
* [demos] added an event demo, also minor cleanup, `#27 <https://github.com/kobuki-base/kobuki_core/pull/27>`_.
* [infra] don't use system headers for dependencies, `#27 <https://github.com/kobuki-base/kobuki_core/pull/27>`_.

1.1.0 (2020-08-04)
------------------
* [infra] ament_export_interfaces -> ament_export_targets, `#21 <https://github.com/kobuki-base/kobuki_core/pull/21>`_.
* [driver] bugfix correct size for pyaload length lookup, #19 <https://github.com/kobuki-base/kobuki_core/pull/19>`_.
* [infra] license links updated, `#15 <https://github.com/kobuki-base/kobuki_core/pull/15>`_.

1.0.0 (2020-01-13)
------------------
* [infra] port and release for ROS2/Eloquent
* [infra] udev rule and debian installation thereof, `#13 <https://github.com/kobuki-base/kobuki_core/pull/13>`_.

0.8.1 (2019-01-09)
------------------
* remove unused and dodgy float builder, fixed uninitialised warnings, `#53 <https://github.com/yujinrobot/kobuki_core/pull/53>`_.

0.8.0 (2019-01-09)
------------------
* updated build infra for changes ament/eloquent

0.7.7 (2017-03-29)
------------------
* accel limiter variable initialisation bugfix

0.7.6 (2017-03-27)
------------------
* doxygen updates

0.7.4 (2017-02-23)
------------------
* drop spurious debugging cmake message that was throwing warnings

0.7.2 (2016-11-09)
------------------
* Pose2D -> LegacyPose2D

0.7.1 (2016-05-06)
------------------
* isnan -> std::isnan

0.6.3 (2016-05-06)
------------------
* bugfix cliff header packet

0.6.2 (2016-01-09)
------------------
* enable the cx11 compiler

0.6.0 (2014-08-04)
------------------
* add linear and angular test plot image
* kobuki_core : Updated doxygen. Issue `#10 <https://github.com/yujinrobot/kobuki_core/issues/10>`_.
* Contributors: Younghun Ju, jihoonl

0.5.3 (2013-09-06)
------------------
* Updated doxygen documentation.

0.5.2 (2013-08-31)
------------------
* Documentation and other text files reviewed.

0.5.1 (2013-08-30)
------------------
* Updated license info.
* Updated doxygen in the code.
* Update firmware_changelog.md.

0.5.0 (2013-08-29)
------------------
* Added debug topic of raw control command of robot.
* Removed debugging message.
* Updated doxygen about factory-default value of PID gain.
* Add firmware version checking on PID commands.
* Updated protocol specification in doxygen about custom PID gain setting.
* Updated doxygen script to let user do something when authentification of repository is failed.
* Bump minor version for firmware (new version is 1.2.0).
* Improved packet integrity checking. Issue `#245 <https://github.com/yujinrobot/kobuki/issues/245>`_.
* Added new protocol about custom PID gain setting. Issue `#249 <https://github.com/yujinrobot/kobuki/issues/249>`_.
* Updated doxygen.
* Removed ros logging code.
* Removed meaningless buffer size check in serialization of command packet.
* Fixed typo; mayor --> major.
* Added integrity check for each packets.
* Removed old printf codes.
* Removed meaningless buffer size check in serialization of packets.
* Fixed typo on update_doxygen.bash script.
* Added extra url info on all packages.
* Added brief description of update_doxygen.bash script.
* Added convenient script for automated update of doxygen document to the github.io page.
* Updated doxygen.
* Updated old rnd email address.
* Removed SetPower command packet from doxygen. It is not for external powers and unnecessary for kobuki.
* Corrected typos on doxygen about external power.
* lock api for protecting data access with asynchronous getXXX calls.
* Fix URL to the previous changelog wiki
* Changelogs at package level
* Reset odometry also for heading
* Updated raw_control_command topic to publish recevied command velocity also.
* Added a debug topic that publish actual base command sent to robot.
* Added simple_keyop application to control kobuki directly from keyboard without ROS.
* Fixed broken synchronity of base control command caused by recent bugfix of acceleration limiter module.
* Added flexible logging features to using named logging system of ros/log4cxx.
* Reset odometry also works for heading (gyro).
* Do not use robot_pose_ekf; use imu for heading and encoders for position.

0.4.0 (2013-08-09)
------------------
* Windows compatible.
* Update doxygen documentation for using wstool and for windows compilation.
* New scripts to help serve firmware and windows downloads from our file server.
* Delay demo program finish so kobuki can beep alive.
* Serial connection made much more robust.
* Install using wstool
* Update firmware_changelog.md with latest verions and rewrite with for markdown formatting.


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/kobuki/ChangeList
