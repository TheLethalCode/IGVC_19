roslaunch pointgrey_camera_driver camera.launch &
sleep 1

while [ 1 ]
do
    if [ $(rosparam get cam) == "0" ]
    then
      printf "Killing camera !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
      killall -9 nodelet
      sleep 1
      # sudo chmod 777 /dev/tty*
      printf "Relaunching camera -----------------------------------------------------------------------"
      roslaunch pointgrey_camera_driver camera.launch &
      sleep 3
    fi
    if [ $(rosparam get kill) == "1" ]
    then 
      killall -9 nodelet
      kill -9 $PPID
      exit 1
    fi
    sleep 1
done


