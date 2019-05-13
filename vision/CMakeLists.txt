cmake_minimum_required(VERSION 2.8.3)
project(vision)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs	
  sensor_msgs
  cv_bridge

)

find_package(OpenCV REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES vision
  #CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(master src/master.cpp)
target_link_libraries(master
       ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
#add_dependencies(master beginner_tutorials_generate_messages_cpp)



