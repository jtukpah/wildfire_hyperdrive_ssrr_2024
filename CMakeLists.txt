cmake_minimum_required(VERSION 3.0.2)
project(imec_driver)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  rospy
  roscpp
  rviz
)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## Use Qt signals and slots to avoid conflicts with boost signals,
## so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

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
   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs roscpp cv_bridge image_transport
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${HEADER_FILES})
else()
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${HEADER_FILES})
endif()

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})