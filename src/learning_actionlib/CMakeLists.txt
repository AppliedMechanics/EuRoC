cmake_minimum_required(VERSION 2.8.3)
project(learning_actionlib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)


add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

add_executable(fibonacci_server src/fibonacci_server.cpp)

target_link_libraries(
  fibonacci_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_server
  ${learning_actionlib_EXPORTED_TARGETS}
)

add_executable(fibonacci_client src/fibonacci_client.cpp)
target_link_libraries(fibonacci_client ${catkin_LIBRARIES})
add_dependencies(fibonacci_client  ${learning_actionlib_EXPORTED_TARGETS})


