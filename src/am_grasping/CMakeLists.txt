cmake_minimum_required(VERSION 2.8.3)
project(am_grasping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  roscpp 
  roslib 
  std_msgs
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


add_executable(am_grasping_srv src/GetGraspPose_server.cpp)
target_link_libraries(am_grasping_srv ${catkin_LIBRARIES} ${Boost_LIBRARIES} crops_manipulator_utils)
add_dependencies(am_grasping_srv ${am_grasping_EXPORTED_TARGETS})

add_executable(am_grasping_client src/Grasping_Dummy_Client.cpp)
target_link_libraries(am_grasping_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(am_grasping_client  ${am_grasping_EXPORTED_TARGETS})