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
  actionlib
  actionlib_msgs
  am_msgs
  sensor_msgs
  crops_manipulator_utils)


# add_action_files(
#   DIRECTORY action
#   FILES test.action
# )

# generate_messages(
#    DEPENDENCIES std_msgs  # Or other packages containing msgs
# )
generate_messages(
  DEPENDENCIES am_msgs actionlib_msgs std_msgs # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS crops_manipulator_utils
)

include_directories(src include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(am_gripper_interface src/main_gripper_interface.cpp)
target_link_libraries(am_gripper_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(am_gripper_interface  am_grasping_generate_messages_cpp)

add_executable(am_grasping src/GraspPose2.cpp src/main_get_grasp_pose.cpp)
target_link_libraries(am_grasping ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(am_grasping  am_grasping_generate_messages_cpp)


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