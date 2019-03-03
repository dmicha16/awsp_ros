#include <ros/ros.h>
#include <ros/console.h>
#include "catamaran_controller/test.h"

int main(int argc, char **argv)
{
    /* code */
    ros::init(argc, argv, "test_node");
    ROS_INFO("Node initialized!");
    ros::NodeHandle n;
    PropellerTest test(n, 17, 27); // Left and Right pins respectively 


    return 0;
}
