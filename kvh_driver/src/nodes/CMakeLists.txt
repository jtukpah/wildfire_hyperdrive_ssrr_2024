#Build Nodes
rosbuild_add_executable(kvh_driver_node kvh_driver_node.cpp)
target_link_libraries(kvh_driver_node ${PROJECT_NAME}Filters)
target_link_libraries(kvh_driver_node ${PROJECT_NAME}Configuration)
rosbuild_add_executable(kvh_test_node   kvh_test_node.cpp)
target_link_libraries(kvh_test_node ${PROJECT_NAME}Filters)