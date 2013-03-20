#Build Tests
rosbuild_add_gtest(test/kvhtest kvhtest.cpp kvh_imu_test.cpp)
target_link_libraries(test/kvhtest ${PROJECT_NAME}Filters ${PROJECT_NAME}Driver)
