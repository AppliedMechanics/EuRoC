cmake_minimum_required(VERSION 2.8.3)
project(learning_actionlib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  am_msgs
  sensor_msgs
  actionlib_msgs
  std_msgs
  actionlib
  crops_manipulator_utils
  message_generation
  roscpp
  rospy
)


add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

generate_messages(
  DEPENDENCIES am_msgs actionlib_msgs # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS am_msgs actionlib_msgs
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

add_executable(goalPose_client src/goalPose_client.cpp)
target_link_libraries(goalPose_client ${catkin_LIBRARIES})
add_dependencies(goalPose_client  ${learning_actionlib_EXPORTED_TARGETS})

add_executable(testing_tf2 src/testing_tf2.cpp)
target_link_libraries(testing_tf2 ${catkin_LIBRARIES})
add_dependencies(testing_tf2  ${learning_actionlib_EXPORTED_TARGETS})
