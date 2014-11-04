cmake_minimum_required(VERSION 2.8.3)
project(am_dyn_tf_broadcaster)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  std_msgs
  euroc_c2_msgs
  actionlib
  actionlib_msgs
  am_msgs
  sensor_msgs
  crops_manipulator_utils
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

generate_messages(
  DEPENDENCIES am_msgs actionlib_msgs euroc_c2_msgs std_msgs # Or other packages containing msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES am_dyn_tf_broadcaster
  CATKIN_DEPENDS am_msgs crops_manipulator_utils euroc_c2_msgs message_generation roscpp roslib std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(src include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})



## Declare a cpp executable

add_executable(am_dyn_tf_broadcaster src/TFBroadcaster.cpp src/main_tf_broadcaster.cpp)
target_link_libraries(am_dyn_tf_broadcaster ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(am_dyn_tf_broadcaster  am_tf_broadcaster_generate_messages_cpp)


################################################################
# INSTALLING

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE)
