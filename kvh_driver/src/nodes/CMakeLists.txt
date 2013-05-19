#Build Nodes
rosbuild_add_executable(kvh_driver_node kvh_driver_node.cpp)
target_link_libraries(kvh_driver_node ${PROJECT_NAME}Driver)