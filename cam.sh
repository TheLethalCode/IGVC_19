trap cleanup 2
cleanup()
{
  killall -9 nodelet
  kill -9 $PPID
  exit 1
}
roslaunch pointgrey_camera_driver camera.launch &
sleep 1

while [ 1 ]
do
    if [ $(rosparam get cam) == "0" ]
    then
      printf "!!!!!!!\nKilling Camera\n!!!!!!!"
      killall -9 nodelet
      sleep 1
      # sudo chmod 777 /dev/tty*
      printf "--------\nRelaunching IMU\n---------"
      roslaunch pointgrey_camera_driver camera.launch &
      sleep 3
    fi
    printf "%d" $(rosparam get cam)
    if [ $(rosparam get kill) == "1" ]
    then 
      killall -9 nodelet
      kill -9 $PPID
      exit 1
    fi
    sleep 1
done


