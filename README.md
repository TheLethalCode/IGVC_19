# IGVC 2019 [Updated 6th June]

Trial branch is the current branch.

1.  "params" folder contains all the parameter files for rqt_reconfigure used during testing, the latest one being 4th_10am.yaml which can be directly imported in rqt_reconfigure. 

2.  The script run_sensors.sh runs all the sensors with appropriate permissions, increases camera buffer and also restarts any sensor node that stops working, cam.sh and bash_edit.py are auxilliary scripts called by run_sensors.sh, they don't have to be started separately.

3.  The file gps_points.txt is used to enter 4 waypoints in the format:  
latitutde1 longitude1  
latitutde2 longitude2  
latitutde3 longitude3  
latitutde4 longitude4  

4.  The file odom_points.txt is used to enter the x and y of the goal in odom frame in the format:  
x_goal y_goal

5.  The file teb_params_no_mans_land.sh contains the TEB parameter changes that are to be made in No Man's Land. Similarly, the file teb_params_vision.sh contains the TEB parameters that are to be used with vision working.


## Order of running nodes - 

1.  ```roscore```
2.  ```bash run_sensor.sh```
3.  ```rostopic echo /sensor_status```
4.  ```rosrun vision master```
5.  ```rosrun rqt_reconfigure rqt_reconfigure```
6.  ```sudo chmod 777 /dev/tty* && roslaunch eklavya4_roboteq manual_launch.launch```
7.  ```roslaunch tf tf.launch```
8.  ```roslaunch robot_localization dual_ekf_navsat.launch```
9.  ```roslaunch huskey_move move.launch```
10.  ```rosrun outdoor_waypoint_nav switchgps```
11.  ```rostopic echo /use_vision```

## Troubleshooting nodes - 

1.  If the run_sensors.sh script is unable to properly run some sensor, stop the script and start the sensors individually as follows:  
  1.1  ```sudo sysctl -w net.core.rmem_max=1048576 net.core.rmem_default=1048576```  
  1.2  ```roslaunch pointgrey_camera_driver camera.launch```  
  1.3  ```roslaunch vn_ins module.launch```  
  1.4  ```sudo chmod 777 /dev/tty* && roslaunch hokuyo_node hokuyo_test.launch```  
  1.5   ```roslaunch pointgrey_camera_driver camera.launch```  
  
2.  While configuring rqt_reconfigure, if any parameter is set that gives a fault in the vision code, for eg, median blur kernel size is set to even, then the vision node won't be able to start as it will reload the last set values.  
In that case, change the parameter from a terminal to another value using -  
  ```rosparam set parameter_name value```  
where the parameter name can be found from ```rosparam list```

3.  The node outdoor_waypoint_nav has to be restarted before every run, since it contains a flag that allows for only one entry into no man's land.
