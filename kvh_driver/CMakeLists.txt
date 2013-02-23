cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
#generate_dynamic_reconfigure_options(cfg/KVHDriver.cfg)
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#build local library
rosbuild_add_library(${PROJECT_NAME}_lib src/helpers/imu_filter.cpp src/helpers/linearfilter.cpp src/helpers/odometryfilter.cpp)
rosbuild_link_boost(${PROJECT_NAME}_lib thread)

#Build Nodes
rosbuild_add_executable(kvh_driver_node src/nodes/kvh_driver_node.cpp)
target_link_libraries(kvh_driver_node ${PROJECT_NAME}_lib)
rosbuild_add_executable(kvh_test_node src/nodes/kvh_test_node.cpp)
target_link_libraries(kvh_test_node ${PROJECT_NAME}_lib)