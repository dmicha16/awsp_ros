//
// Created by davidm on 4/27/19.
//
#include "awsp_controller/dynamic_parameters.h"

#ifndef PROJECT_STATE_MACHINE_H
#define PROJECT_STATE_MACHINE_H

namespace state {

const int SYSTEM_OFF = 1;
const int POSE_ESTIMATION = 2;
const int GOAL_SETTING = 3;
const int BOAT_CONTROLLER = 4;

int current_system_state = 1;

void print_system_off_status() {
    ROS_WARN_STREAM("[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM("[SYSTEM IS IN STATE     ] 1 - SYSTEM_OFF");
    ROS_DEBUG_STREAM("[LINEAR GAIN            ] " << dynr::control_gains.linear_gain);
    ROS_DEBUG_STREAM("[ANGULAR GAIN           ] " << dynr::control_gains.angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);
//    ROS_DEBUG_STREAM();
    ROS_DEBUG("================================================");
}

void print_pose_estimation_status() {
    ROS_WARN_STREAM("[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM("[SYSTEM IS IN STATE     ] 2 - POSE_ESTIMATION");
    ROS_DEBUG_STREAM("[LINEAR GAIN            ] " << dynr::control_gains.linear_gain);
    ROS_DEBUG_STREAM("[ANGULAR GAIN           ] " << dynr::control_gains.angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);

    ROS_DEBUG("================================================");
}

void print_goal_setting_status() {
    ROS_WARN_STREAM("[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM("[SYSTEM IS IN STATE     ] 3 - GOAL_SETTING");
    ROS_DEBUG_STREAM("[CURRENT GOAL LATITUDE  ] " << dynr::current_vessel_task.goal_latitude);
    ROS_DEBUG_STREAM("[CURRENT GOAL LONGITUDE ] " << dynr::current_vessel_task.goal_longitude);
    ROS_DEBUG_STREAM("[LINEAR GAIN            ] " << dynr::control_gains.linear_gain);
    ROS_DEBUG_STREAM("[ANGULAR GAIN           ] " << dynr::control_gains.angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);

    ROS_DEBUG("================================================");
}

void print_boat_controller_status() {
    ROS_WARN_STREAM("[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM("[SYSTEM IS IN STATE     ] 4 - BOAT_CONTROLLER");
    ROS_DEBUG_STREAM("[CURRENT GOAL LATITUDE  ] " << dynr::current_vessel_task.goal_latitude);
    ROS_DEBUG_STREAM("[CURRENT GOAL LONGITUDE ] " << dynr::current_vessel_task.goal_longitude);

    /*
     * distance to goal
     * current velocity estimate
     * current gps estimate
     * current position estimate
     * etc.
     */


    ROS_DEBUG_STREAM("[LINEAR GAIN            ] " << dynr::control_gains.linear_gain);
    ROS_DEBUG_STREAM("[ANGULAR GAIN           ] " << dynr::control_gains.angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);

    ROS_DEBUG("================================================");
}

int evaluate_system_mode_status()
{
    if (dynr::system_mode.vessel == OFF) {
        return SYSTEM_OFF;
    }
}

void system_off() {
    if (dynr::system_mode.vessel == OFF) {
//        ROS_WARN("The system is now TURNING OFF.");
//        ROS_INFO("The system is currently OFF, waiting for initialization.");
        ros::Rate loop_rate(10);

        state::print_system_off_status();

        while (ros::ok()) {
            // print status while we are waiting for the initial startup

            if (dynr::system_mode.vessel == ON) {
                ROS_WARN("The system is now TURNING ON.");
                break;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}

int pose_estimation() {
    int move_to_next = 0;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        state::print_pose_estimation_status();
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
            return state::SYSTEM_OFF;

        if (dynr::debugging.log_imu_raw == true)
            return state::GOAL_SETTING;

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int goal_setting() {
    int move_to_next = 0;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        state::print_goal_setting_status();
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
            return state::SYSTEM_OFF;
        if (dynr::debugging.log_imu_kalman == true)
            return state::BOAT_CONTROLLER;

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int boat_controller() {
    int move_to_next = 0;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        state::print_boat_controller_status();
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
            return state::SYSTEM_OFF;

        if (dynr::debugging.log_gps_raw == true)
            return state::POSE_ESTIMATION;

        ros::spinOnce();
        loop_rate.sleep();
    }
}

}

#endif //PROJECT_STATE_MACHINE_H
