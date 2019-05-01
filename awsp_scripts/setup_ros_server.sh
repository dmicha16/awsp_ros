#!/usr/bin/env bash
echo "--> This script is to setup the SERVER to be able to connect to remote a ROS master."
HOSTNAME=`hostname -I`
HOSTNAME_S="$(echo "${HOSTNAME}" | tr -d '[:space:]')"
export ROS_MASTER_URI=http://$HOSTNAME_S:11311
export ROS_IP=$HOSTNAME_S
echo "--> All environment variables are now set, gonna print those variables now!"
printenv | grep ROS_MASTER
printenv | grep ROS_IP
#echo "--> Gonna run 'roswtf' now, make sure you don't see any errors pop up!"
#sleep 3
##roswtf