cmake_minimum_required(VERSION 2.8.3)

# package name 
project(sensor_status)

# Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_C_FLAGS "-w" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	image_transport 
	roscpp
	std_msgs
    message_generation
	rospy
)
add_message_files(
  FILES
  sensor_status.msg
 ) 
find_package(OpenCV REQUIRED)
generate_messages(
   DEPENDENCIES
   std_msgs
 )
catkin_package(
CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
)

add_executable(error src/error_log.cpp)


target_link_libraries(error 
  ${catkin_LIBRARIES}
)
add_dependencies(error sensor_status_generate_messages_cpp)

