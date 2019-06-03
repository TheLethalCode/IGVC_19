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
      printf "\n!!!!!!!\nKilling Camera\n!!!!!!!\n"
      killall -9 nodelet
      sleep 1
      printf "\n--------\nRelaunching Camera\n---------\n"
      roslaunch pointgrey_camera_driver camera.launch &
      sleep 3
    fi
    # printf "%d" $(rosparam get cam)
    if [ $(rosparam get kill) == "1" ]
    then 
      killall -9 nodelet
      kill -9 $PPID
      exit 1
    fi
    sleep 1
done


