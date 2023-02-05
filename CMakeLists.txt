cmake_minimum_required(VERSION 3.0.2)
project(imec_driver)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  roscpp
  message_generation
)

add_message_files(
   FILES
   DataCube.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  adjust_param.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imec_driver
#  CATKIN_DEPENDS cv2 cv_bridge numpy os rospy sensor_msgs.msg
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# include_directories(/opt/imec/hsi-mosiac/include)
# link_directories(/opt/imec/hsi-mosiac/bin)

# find_library(HSI_CAMERA_API imec REQUIRED)
# if (NOT HSI_CAMERA_API)
#     message(FATAL_ERROR "hsi_camera_api library not found")
# endif()

# if(HSI_CAMERA_API) 
#   message(FATAL_ERROR "Found ya boi")
# endif()

# set(hsi_camera_api "/opt/imec/hsi-mosiac/bin/libhsi_camera_api.so")  # the full path. if postfix is `.so`, change it

add_executable(grab_datacube src/grab_datacube.cpp)

# target_include_directories(grab_datacube PUBLIC "/opt/imec/hsi-mosiac/include/hsi_camera_api.h")
# target_link_directories(grab_datacube PUBLIC "/opt/imec/hsi-mosiac/bin/libhsi_camera_api.so")

file(GLOB SO_FILES "${PROJECT_SOURCE_DIR}/lib/*.so")

target_link_libraries(grab_datacube ${catkin_LIBRARIES} ${SO_FILES})
# target_link_libraries(grab_datacube 
#    ${catkin_LIBRARIES} /opt/imec/hsi-mosiac/bin/libhsi_camera_api.so
# )
# target_include_directories(grab_datacube PUBLIC
#   "/opt/imec/hsi-mosiac/include"
# )
