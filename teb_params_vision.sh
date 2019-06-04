rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS xy_goal_tolerance 0.3
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS yaw_goal_tolerance 0.3 #18 degrees on either side
cmb = $(rosparam get /master/costmap_median_blur)
rosrun dynamic_reconfigure dynparam set /master costmap_median_blur $cmb
