rosbuild_add_executable(serialtest serialtest.cpp)
target_link_libraries(serialtest ${PROJECT_NAME}Driver)

rosbuild_add_executable(stream_viewer stream_viewer.cpp)
target_link_libraries(stream_viewer ${PROJECT_NAME}Driver)

rosbuild_add_executable(bit bit.cpp)
target_link_libraries(bit ${PROJECT_NAME}Driver)

rosbuild_add_executable(config config.cpp)
target_link_libraries(config ${PROJECT_NAME}Driver)

rosbuild_add_executable(drift_test drift_test.cpp)
target_link_libraries(drift_test ${PROJECT_NAME}Driver)
