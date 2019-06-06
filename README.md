# IGVC 2019 [Updated 6th June]

Trial branch is the current branch.

1. "params" folder contains all the parameter files for rqt_reconfigure used during testing, the latest one being 4th_10am.yaml which can be directly imported in rqt_reconfigure. 

2. The script run_sensors.sh runs all the sensors with appropriate permissions, increases camera buffer and also restarts any sensor node that stops working, cam.sh and bash_edit.py are auxilliary scripts called by run_sensors.sh, they don't have to be started separately.

3. The file gps_points.txt is used to enter 4 waypoints in the format:

latitutde1 longitude1  
latitutde2 longitude2  
latitutde3 longitude3  
latitutde4 longitude4  

4. The file odom_points.txt is used to enter the x and y of the goal in odom frame in the format:

x_goal y_goal

5. The file teb_params_no_mans_land.sh contains the TEB parameter changes that are to be made in No Man's Land. Similarly, the file teb_params_vision.sh contains the TEB parameters that are to be used with vision working.
