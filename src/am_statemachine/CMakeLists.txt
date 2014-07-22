cmake_minimum_required(VERSION 2.8.3)
project(statemachine)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES std_msgs)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(statemachine src/main_statemachine.cpp)
target_link_libraries(statemachine ${catkin_LIBRARIES})
#add_dependencies(statemachine statemachine_generate_messages_cpp)

add_executable(listener src/main_listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener statemachine_generate_messages_cpp)

