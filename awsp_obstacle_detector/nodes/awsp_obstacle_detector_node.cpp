//
// Created by davidm on 5/8/19.
//
#include <ros/ros.h>
#include "awsp_msgs/ObstacleData.h"
#include <wiringPi.h>
#include <iostream>
#include <chrono>

int main(int argc, char **argv)
{
    ROS_INFO("Starting node");
    ros::init(argc, argv, "awsp_obstacle_detector_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<awsp_msgs::ObstacleData>("obstacle_data", 1000);
    awsp_msgs::ObstacleData obstacle_data;
    ros::Rate rate(100);

    if (wiringPiSetup() != 0)
    {
        printf("Failed to wiringPiSetupGpio()\n");
        return 0;
    }

    int trigger_ = 23;
    int echo_ = 24;
    pinMode(trigger_, OUTPUT);
    pinMode(echo_, INPUT);
    delay(2000);

    long end_time = 0;
    long now, travelTimeUsec, timeout;
    volatile long startTimeUsec;
    volatile long endTimeUsec;
    double distanceMeters;

    while(ros::ok())
    {
        digitalWrite(trigger_, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_, LOW);

        now=micros();
        timeout = 30000;

        while (digitalRead(echo_) == LOW && micros()-now<timeout);
        startTimeUsec = micros();
        while ( digitalRead(echo_) == HIGH );
        endTimeUsec = micros();

        travelTimeUsec = endTimeUsec - startTimeUsec;
        distanceMeters = 100*((travelTimeUsec/1000000.0)*340.29)/2;

        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}