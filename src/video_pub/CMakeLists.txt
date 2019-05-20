cmake_minimum_required(VERSION 2.8.3)
project(video_pub)

find_package(OpenCV 3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs	
  sensor_msgs
  cv_bridge
  image_transport
)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES vision
  #CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)


include_directories(  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_publisher src/video_publisher.cpp)

target_link_libraries(video_publisher
       ${catkin_LIBRARIES}
       ${OpenCV_LIBRARIES}
)

add_dependencies(video_publisher ${catkin_EXPORTED_TARGETS})