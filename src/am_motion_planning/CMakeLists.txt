cmake_minimum_required(VERSION 2.8.3)
project(am_motion_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  am_msgs
  actionlib
  roscpp
  crops_manipulator_utils
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface	
  pluginlib
  cmake_modules
)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES am_motion_planning
#  CATKIN_DEPENDS am_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	src
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(am_motion_planning src/T6MotionPlanning.cpp src/MotionPlanning.cpp src/main_motion_planning.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(am_motion_planning am_motion_planning_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(am_motion_planning
   ${catkin_LIBRARIES}
   crops_manipulator_utils
 )


