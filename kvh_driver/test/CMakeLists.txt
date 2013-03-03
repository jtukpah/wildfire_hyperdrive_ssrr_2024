#Build Tests
rosbuild_add_gtest(test/kvhtest kvhtest.cpp)
target_link_libraries(test/kvhtest ${PROJECT_NAME}Filters)