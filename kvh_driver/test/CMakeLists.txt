#Build Tests
rosbuild_add_gtest(test/kvhtest kvhtest.cpp kvhconfigtest.cpp)
target_link_libraries(test/kvhtest ${PROJECT_NAME}Filters)
target_link_libraries(test/kvhtest ${PROJECT_NAME}Configuration)
