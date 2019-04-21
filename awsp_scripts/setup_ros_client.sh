#!/usr/bin/env bash
#  - To actually make it work and connenct other running my two scripts on both sides of the client and server, it is also might be needed to set the `/etc/hosts` file with this line:
#  - `{SERVER IP} {SERVER_NAME}`
#  - to get the ip that goes in there do: `hostname -I`
#  - to get the server name: `whoami` (or in some cases `awsp_pi`)
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