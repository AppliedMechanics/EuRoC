# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(crops_manipulator_utils)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# collect my source files
set(MY_LIB_SOURCES
  src/tf_rot.cpp
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
  DEPENDS
  CATKIN_DEPENDS std_msgs roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

# build the crops_manipulator_utils lib
add_library(${PROJECT_NAME} ${MY_LIB_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## build a node to write the contents of a joint_trajectory message into a textfile
#add_executable(jt2textfile src/joint_trajectory2inputfile.cpp)
#target_link_libraries(jt2textfile ${catkin_LIBRARIES})

## ...
#add_executable(pub_joint_states src/publish_joint_states.cpp)
#add_dependencies(pub_joint_states ${catkin_EXPORTED_TARGETS})
#target_link_libraries(pub_joint_states ${catkin_LIBRARIES})

# node to draw markers in rviz (from the manipulator_target_obstacle message)
# add_executable(draw_obstacles
#   src/main_draw_target_obstacles.cpp
#   include/tf_rot.cpp
#   src/RvizMarker.cpp)
# add_dependencies(draw_obstacles ${catkin_EXPORTED_TARGETS})


#install targets (not required)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#         PATTERN ".svn" EXCLUDE)




# <export> <cpp cflags="-I${prefix}/include" lflags="-L${prefix}/lib -lcrops_manipulator_utils"/> </export>

