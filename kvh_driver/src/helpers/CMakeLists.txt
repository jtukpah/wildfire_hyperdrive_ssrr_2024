#build local libraries
rosbuild_add_library(${PROJECT_NAME}Filters
		      imu_filter.cpp
		      linearfilter.cpp
		      odometryfilter.cpp)
		      
rosbuild_add_library(${PROJECT_NAME}Configuration
		      configurations.cpp)
		      
rosbuild_link_boost(${PROJECT_NAME}Filters thread)