# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(am_gui)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs crops_manipulator_utils)

# qt compatibility added by J. Baur
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui Qtnetwork)

#set definitions for Qt
ADD_DEFINITIONS(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_WEBKIT)
ADD_DEFINITIONS(-DQT_NO_DEBUG)
ADD_DEFINITIONS(-DQT_SHARED)
ADD_DEFINITIONS(-DQT_GUI_LIB)
ADD_DEFINITIONS(-DQT_NETWORK_LIB)
ADD_DEFINITIONS(-DQT_CORE_LIB)

#specify the include dirs
set(QT_MY_INCLUDES
  ${QT_INCLUDE_DIR}
  ${QT_QTNETWORK_INCLUDE_DIR}
  ${QT_QTTEST_INCLUDE_DIR}
  src
  src/rosinterface
  src/gui
  src/gui/components
  )

# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#qt
#define qt sources
set(QT_SOURCES
  src/gui/mainwindow.cpp
  src/gui/manipulatorstatuswidget.cpp
  src/gui/simplecontrolwidget.cpp
  src/gui/telemetrywidget.cpp
  src/rosinterface/ROSinterface.cpp
)

#define qt headers
set(QT_HEADERS
  src/gui/mainwindow.h
  src/gui/manipulatorstatuswidget.hpp
  src/gui/simplecontrolwidget.hpp
  src/gui/telemetrywidget.hpp
  src/rosinterface/ROSinterface.hpp
)

#define moc headers
QT4_WRAP_CPP(QT_MOC_HPP ${QT_HEADERS})
include(${QT_USE_FILE})

# set include directories
include_directories(include
  ${QT_MY_INCLUDES}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

# collect my source files
set(MY_SOURCES
  src/main_gui.cpp
  ${QT_SOURCES}
  ${QT_MOC_HPP})


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs crops_manipulator_utils
  INCLUDE_DIRS #include
  LIBRARIES # TODO
  )

#build the executable
add_executable(${PROJECT_NAME} ${MY_SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#link with qt libs
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${QT_QTNETWORK_LIBRARIES}
  ${QT_QTTEST_LIBRARIES}
  ${QT_GUI_LIB}
  ${QT_CORE_LIB}
  ${catkin_LIBRARIES})

#install targets (not required)
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
