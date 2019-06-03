trap cleanup 2
off=0
rosparam set lid 0
rosparam set imu 0
rosparam set cam 0

cleanup()
{
  echo "Caught Signal ... cleaning up."
  rosparam set kill 1  
  killall -9 error
  sleep 1
  killall -9 hokuyo_node
  sleep 1
  killall -9 vn_ins
  sleep 1
  killall -9 nodelet
  sleep 1
  # kill -9 $PPID
  exit 1
}

python bash_edit.py
rosparam set kill 0

sudo chmod 777 /dev/serial/by-id/*
sudo sysctl -w net.core.rmem_max=1048576 net.core.rmem_default=1048576

rosrun rosserial_python serial_node.py /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0

rosrun sensor_status error &

roslaunch vn_ins module.launch &
sleep 1

roslaunch hokuyo_node hokuyo_test.launch &
sleep 1

#parallel launch of camera node
terminator -e "env INIT_CMD='bash cam.sh' bash " &

while [ 1 ]
do
    if [ $(rosparam get imu) == "0" ]
    then
        printf "\n!!!!!!!\nKilling IMU\n!!!!!!!\n"
        killall -9 vn_ins
        sleep 1
        sudo chmod 777 /dev/serial/by-id/*
        printf "\n--------\nRelaunching IMU\n---------\n"
        roslaunch vn_ins module.launch &
        sleep 1
    fi
    
    if [ $(rosparam get lid) == "0" ]
    then
      printf "\n!!!!!!!!\nKilling Lidar\n!!!!!!!!!\n"
      killall -9 hokuyo_node
      sleep 1
      sudo chmod 777 /dev/serial/by-id/*
      printf "\n--------\nRelaunching Lidar\n--------\n"
      roslaunch hokuyo_node hokuyo_test.launch &  
      sleep 1
    fi
    #if [ $(rosparam get cam) == "0" ]
    #then
    #  printf "!!!!!!!!!\nKilling camera\n!!!!!!!!"
    #  killall -9 nodelet
    #  sleep 1
    #  # sudo chmod 777 /dev/serial/by-id/*
    #  printf "--------\nRelaunching camera\n--------"
    #  roslaunch pointgrey_camera_driver camera.launch &
    #  sleep 2
    #fi

    if [ $(rosparam get imu) == "1" ] && [ $(rosparam get cam) == "1" ] && [ $(rosparam get lid) == "1" ] 
    then
        printf "\n........\nALL CONNECTED\n.........\n"
    fi
    sleep 3
done



