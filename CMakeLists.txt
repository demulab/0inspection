cmake_minimum_required(VERSION 2.8.3)
project(inspection)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
 rospy roscpp roslib diagnostic_msgs message_generation actionlib geometry_msgs move_base_msgs nav_msgs std_msgs tf urg_node sensor_msgs joy OpenCV )
find_package(PCL 1.3 REQUIRED COMPONENTS common io system )

## add demu
include_directories(/usr/local/include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(/usr/local/lib ${PCL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES inspection
  CATKIN_DEPENDS roscpp rospy std_msgs roslib geometry_msgs 
  DEPENDS system_lib urg_node sensor_msgs roscpp 
)

###########
## Build ##
###########

add_executable(inspection src/inspection.cpp)
target_link_libraries(inspection ${catkin_LIBRARIES} libkobuki.so ${OpenCV_LIBRARIES} )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/move
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS inspection
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
