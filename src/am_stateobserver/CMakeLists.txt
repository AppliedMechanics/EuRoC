cmake_minimum_required(VERSION 2.8.3)
project(am_stateobserver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  roscpp 
  roslib 
  std_msgs
  am_msgs
  crops_manipulator_utils)


# add_action_files(
#   DIRECTORY action
#   FILES test.action
# )

generate_messages(
  DEPENDENCIES std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS crops_manipulator_utils
)

include_directories(src include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(am_stateobserver src/state_observer.cpp src/main_stateobserver.cpp)
target_link_libraries(am_stateobserver ${catkin_LIBRARIES} ${Boost_LIBRARIES} crops_manipulator_utils)
add_dependencies(am_stateobserver  ${am_stateobserver_EXPORTED_TARGETS})
