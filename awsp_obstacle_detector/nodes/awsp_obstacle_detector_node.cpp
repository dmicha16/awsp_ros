//
// Created by davidm on 5/8/19.
//
#include <ros/ros.h>
#include "awsp_msgs/ObstacleData.h"
#include <wiringPi.h>
#include <iostream>
#include <chrono>

bool evaluate_obstacle(double front_obstacle_dist, float distance_thresh)
{
    if (front_obstacle_dist < 0)
        return false;

    else if (front_obstacle_dist > distance_thresh)
        return false;
    else if (front_obstacle_dist > 0 && front_obstacle_dist < distance_thresh)
        return true;
    else
        return true;
}

int main(int argc, char **argv)
{
    // args TRIGGER, ECHO pins and DISTANCE_THRESH
    if (argc < 2)
    {
        ROS_ERROR("TRIGGER pin not specified.");
        return 1;
    }
    else if (argc < 3)
    {
        ROS_ERROR("ECHO pin not specified.");
        return 1;
    }
    else if (argc < 4)
    {
        ROS_ERROR("Timeout not specified");
        return 1;
    }

    int trigger = atoi(argv[1]);
    int echo = atoi(argv[2]);
    float distance_thresh = atof(argv[3]);
    long timeout = atol(argv[4]);

    ROS_INFO("Starting node");
    ros::init(argc, argv, "awsp_obstacle_detector_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<awsp_msgs::ObstacleData>("obstacle_data", 1000);
    awsp_msgs::ObstacleData obstacle_data;
    ros::Rate rate(10);

    if (wiringPiSetupGpio() != 0)
    {
        ROS_ERROR("Failed to wiringPiSetup()\n");
    }

    ROS_INFO("pint pinMode");
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    delay(2000);
    ROS_INFO("pint pinMode2");

    long now, travel_time_micro;
    volatile long start_time_micro;
    volatile long end_time_micro;
    double front_obstacle_dist;

    while(ros::ok())
    {
        ROS_INFO_ONCE("Started advertising on topic: obstacle_data");
        digitalWrite(trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger, LOW);

        now = micros();

        while (digitalRead(echo) == LOW && micros() - now < timeout);
        start_time_micro = micros();
        while ( digitalRead(echo) == HIGH );
        end_time_micro = micros();

        travel_time_micro = end_time_micro - start_time_micro;
        front_obstacle_dist = ((travel_time_micro/1000000.0)*340.29)/2;

        obstacle_data.front_obstacle_dist = front_obstacle_dist;
        obstacle_data.front_obstacle = evaluate_obstacle(front_obstacle_dist, distance_thresh);

        publisher.publish(obstacle_data);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}