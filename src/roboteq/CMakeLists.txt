cmake_minimum_required(VERSION 2.8.3)
project(eklavya4_roboteq)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  control.msg
  diagnose_msg.msg
 ) 

 find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-std=c++0x)

 add_service_files(
   FILES
   SetSpeed.srv
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES eklavya4_roboteq
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  #DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)



 add_executable(roboteq_server src/roboteq_server.cpp src/RoboteqDevice.cpp)
 target_link_libraries(roboteq_server ${catkin_LIBRARIES})
add_dependencies(roboteq_server eklavya4_roboteq_gencpp)

 add_executable(roboteq_client src/command_subscriber_roboteq_client.cpp)

 target_link_libraries(roboteq_client
   ${catkin_LIBRARIES}
 )
add_dependencies(roboteq_client eklavya4_roboteq_gencpp)


add_executable(joy2 src/joy2.cpp)
target_link_libraries(joy2 ${catkin_LIBRARIES})
add_dependencies(joy2 eklavya4_roboteq_generate_messages_cpp)

add_executable(manual src/manual.cpp)
target_link_libraries(manual ${catkin_LIBRARIES})
add_dependencies(manual eklavya4_roboteq_generate_messages_cpp)


## Specify libraries to link a library or executable target against
