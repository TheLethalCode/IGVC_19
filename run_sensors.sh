trap cleanup 2
port=0
off=0
rosparam set lid 0
rosparam set imu 0
rosparam set cam 0

cleanup()
{
  echo "Caught Signal ... cleaning up."
  killall -9 error
  sleep 1
  killall -9 hokuyo_node
  sleep 1
  killall -9 vn_ins
  sleep 1
  killall -9 nodelet
  sleep 1
  exit 1
}

switch_port () {
    if [ $port \> 1 ]
    then
        port=0
        printf "hello"
    fi
    temp="/dev/ttyACM$port"
    rosparam set port $temp
    printf "port: %d concat:%s\n" $port $temp
    port=$((port+1))
}

if [ $(rosparam get imu) == "0" ]
then
  printf "in" 
fi

sudo chmod 777 /dev/tty*
rosrun sensor_status error &

roslaunch vn_ins module.launch &
sleep 1

sudo sysctl -w net.core.rmem_max=1048576 net.core.rmem_default=1048576 
roslaunch pointgrey_camera_driver camera.launch &
sleep 1

roslaunch hokuyo_node hokuyo_test.launch &
sleep 1

while [ 1 ]
do
    if [ $(rosparam get imu) == "0" ]
    then
        printf "Killing IMU !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        killall -9 vn_ins
        sleep 1
        sudo chmod 777 /dev/tty*
        printf "Relaunching IMU -----------------------------------------------------------------------"
        roslaunch vn_ins module.launch &
        sleep 1
    fi
    
    if [ $(rosparam get lid) == "0" ]
    then
      printf "Killing Lidar !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
      killall -9 hokuyo_node
      sleep 1
      sudo chmod 777 /dev/tty*
      printf "Relaunching Lidar -----------------------------------------------------------------------"
      switch_port
      roslaunch hokuyo_node hokuyo_test.launch &  
      sleep 1
    fi

    if [ $(rosparam get cam) == "0" ]
    then
      printf "Killing camera !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
      killall -9 nodelet
      sleep 1
      sudo chmod 777 /dev/tty*
      printf "Relaunching camera -----------------------------------------------------------------------"
      sudo sysctl -w net.core.rmem_max=1048576 net.core.rmem_default=1048576
      roslaunch pointgrey_camera_driver camera.launch &
      sleep 2
    fi
        
    # cam=$(rosparam get cam)
    # imu=$(rosparam get imu)
    # lid=$(rosparam get lid)
    if [ $(rosparam get imu) == "1" ] && [ $(rosparam get cam) == "1" ] && [ $(rosparam get lid) == "1" ] 
    then
        printf "ALL connected\n"
    fi
    sleep 5
done


