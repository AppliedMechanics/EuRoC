cmake_minimum_required(VERSION 2.8.3)
project(am_statemachine)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs
  message_generation
  roscpp 
  roslib 
  std_msgs)


add_action_files(
  DIRECTORY action
  FILES test.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

##include_directories(src include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(statemachine src/main_statemachine.cpp)
target_link_libraries(statemachine ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(statemachine  ${am_statemachine_EXPORTED_TARGETS})

add_executable(test_node src/main_testnode.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_node  ${am_statemachine_EXPORTED_TARGETS})