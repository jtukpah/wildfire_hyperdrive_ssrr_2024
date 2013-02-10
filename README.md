kvh
========

ROS driver for KVH IMU's

Development
-----------
Checkout the repository to the src directory of the catkin worksapce (`${catkin_workspace}/src/kvh`). To build the packages return to the catkin workspace directory and then run `catkin_make`. To do a clean build run `rm -r build devel`.

The development branch is the 'in-flux' version of the master branch. Code which has been at least tested for basic functionality, but not for full validation, should be merged into this branch.

When creating/modify data for the development branch, create a new 'topic' branch off of development that will contain all of your commits as you work on your modifcations. Once your work is finished and gone through basic testing, merge it back into the development branch.


Packages
--------
- **kvh_driver**: ROS driver for KVH IMU's (rosbuild)
