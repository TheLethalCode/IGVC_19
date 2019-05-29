Make sure you are using Gazebo 7 before proceeding.

Run the following commands:
1. sudo apt-get install ros-<distro>-husky-simulator
2. sudo apt-get install ros-<distro>-husky-desktop
3. export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
4. sudo apt-get install ros-kinetic-dwa-local-planner
5. sudo apt-get install ros-kinetic-move-base
6. sudo apt-get install ros-kinetic-move-base-msgs
7. sudo apt-get install ros-kinetic-rviz

Go to opt/ros/kinetic/share/husky_gazebo/launch and open playpen.launch

Change line 29
	<arg name="world_name" value="$(find husky_gazebo)/worlds/clearpath_playpen.world"/>
	to
	<arg name="world_name" value="$(find husky_gazebo)/worlds/IGVC.world"/>

Paste IGVC.world in opt/ros/kinetic/share/husky_gazebo/worlds

Run the following commands on three different terminals

roslaunch husky_gazebo husky_playpen.launch
roslaunch husky_viz view_robot.launch
roslaunch husky_navigation move_base_mapless_demo.launch

And there you go :-D
