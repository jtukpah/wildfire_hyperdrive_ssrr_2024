#build local libraries
rosbuild_add_library(${PROJECT_NAME}Filters
		      imu_filter.cpp
		      linearfilter.cpp
		      odometryfilter.cpp)
rosbuild_link_boost(${PROJECT_NAME}Filters thread)

rosbuild_add_library(${PROJECT_NAME}Driver kvh_imu.cpp)

rosbuild_add_library(${PROJECT_NAME}Configuration configurations.cpp)
