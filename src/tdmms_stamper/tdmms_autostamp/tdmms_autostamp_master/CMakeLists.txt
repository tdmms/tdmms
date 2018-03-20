##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(tdmms_autostamp_master)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp
  actionlib
  actionlib_msgs
  message_generation
  rospy
  std_msgs
  geometry_msgs
  halcon_bridge
  transfer_linear_yamaha
 )

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tdmms_autostamp_flake_positioner_action
  CATKIN_DEPENDS halcon_bridge transfer_linear_yamaha
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /opt/halcon/include/halconcpp
  /opt/halcon/include
)
##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui QtSql QtDialog QtWidgets) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/tdmms_autostamp_master/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
FIND_LIBRARY(HALCON_LIBRARY halcon /opt/halcon/lib/x64-linux)
FIND_LIBRARY(HALCONCPP_LIBRARY halconcpp /opt/halcon/lib/x64-linux)

add_executable(tdmms_autostamp_master ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(tdmms_autostamp_master ${QT_LIBRARIES} ${catkin_LIBRARIES}   ${HALCON_LIBRARY}
  ${HALCONCPP_LIBRARY}
)
install(TARGETS tdmms_autostamp_master RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

