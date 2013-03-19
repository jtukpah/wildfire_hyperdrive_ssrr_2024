rosbuild_add_executable(serialtest serialtest.cpp)
target_link_libraries(serialtest ${PROJECT_NAME}Driver)
