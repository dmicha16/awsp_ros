#!/usr/bin/env bash
echo "--> This script is to setup the SERVER to be able to connect to remote a ROS master."
export ROS_MASTER_URI=http://$(hostname -I):11311
export ROS_IP=$(hostname -I)
echo "--> All environment variables are now set, gonna print those variables now!"
sleep 2
printenv | grep ROS_MASTER
printenv | grep ROS_IP
echo "--> Gonna run 'roswtf' now, make sure you don't see any errors pop up!"
sleep 3
roswtf