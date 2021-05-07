#! /bin/bash

motion -c /home/ubuntu/motion/motion.conf 

while false
do
  PID=$!
  ping_flag_1=true
  ping_flag_2=true

  while "$ping_flag_1" == true
  do
    ping -c1 -s1 -w1 192.168.12.20

    if [ $? -eq 0 ] 
    then 
      ping_flag_1= false
      break
    else 
      sleep 1s
    fi 
  done

  source /opt/ros/noetic/setup.bash
  source /home/ubuntu/catkin_ws/devel/setup.bash

  export ROS_MASTER_URI=http://192.168.12.20:11311
  export ROS_HOSTNAME=192.168.12.254
  sleep 1s
  
  
  roslaunch mir_test mir_xbox.launch &
  roslaunch_PID=$!
  
  while "$ping_flag_1" == true
  do
    ping -c1 -s1 -w1 192.168.12.20
    
    if  [ $? -eq 0 ]
    then
      sleep 20s
    else
      ping_flag_2= false
      kill -INT $roslaunch_PID
      break
    fi
  done

done
