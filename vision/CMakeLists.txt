cmake_minimum_required(VERSION 2.8.3)
project(vision)

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

add_executable(master src/master.cpp)


target_link_libraries(master
       ${catkin_LIBRARIES}
	   ${OpenCV_LIBRARIES}
)

  add_dependencies(master ${catkin_EXPORTED_TARGETS})

#add_executable(test_vision src/test.cpp)


#target_link_libraries(test_vision
#       ${catkin_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#)

#add_dependencies(test_vision ${catkin_EXPORTED_TARGETS})
#add_executable(lidar src/lidar.cpp)


#target_link_libraries(lidar
#       ${catkin_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#)

#add_dependencies(lidar ${catkin_EXPORTED_TARGETS})