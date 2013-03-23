#Build Tests
rosbuild_add_gtest(kvhtest kvhtest.cpp kvh_imu_test.cpp kvhconfigtest.cpp)
target_link_libraries(kvhtest ${PROJECT_NAME}Filters ${PROJECT_NAME}Driver ${PROJECT_NAME}Configuration)
