#!/usr/bin/env bash
ros_master_ip=$1
echo "--> This script is to setup the CLIENT to be able to connect to remote a ROS master."
#current_ip=$(hostname -I)
export ROS_MASTER_URI=http://$ros_master_ip:11311
export ROS_IP=$(hostname -I)
echo "--> All environment variables are now set, gonna print those variables now!"
sleep 2
printenv | grep ROS_MASTER
printenv | grep ROS_IP
echo "--> Gonna run 'roswtf' now, make sure you don't see any errors pop up!"
sleep 3
roswtf