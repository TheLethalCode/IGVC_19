rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS xy_goal_tolerance 1
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS yaw_goal_tolerance 0.7853981634 #45 degrees on either side
rosrun dynamic_reconfigure dynparam set /master costmap_median_blur 7
