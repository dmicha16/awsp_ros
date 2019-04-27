//
// Created by davidm on 4/27/19.
//
#include "awsp_controller/dynamic_parameters.h"
#include "awsp_pose_estimator/awsp_pose_estimator.h"

#ifndef PROJECT_STATE_MACHINE_H
#define PROJECT_STATE_MACHINE_H

gps_position gps_data;
imu_data imu_data;
bool new_imu = false;
bool new_gps = false;

coordinates_2d vel;
coordinates_2d acc;
CartesianPose pose_estimator(gps_data, gps_data, vel, acc, 0);
cart_pose current_pose;

// References
gps_position gps_ref;
cart_pose cartesian_ref;

// Errors
coordinates_2d cartesian_error;
float distance_error;
float bearing_goal;
float bearing_error;

// Forces and torques
float force_drive;
float torque_drive;
float force_left;
float force_right;

// Speeds
float linear_speed;
float angular_speed;

// Signals
int pwm_left;
int pwm_right;

bool is_first_gps = true;
bool ref_set = false;

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

void print_goal_setting_status()
{
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

void print_boat_controller_status()
{
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

void system_off()
{
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

int pose_estimation()
{


    int move_to_next = 0;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        state::print_pose_estimation_status();
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
            return state::SYSTEM_OFF;

        if (dynr::debugging.log_imu_raw == true)
            return state::GOAL_SETTING;

        if (!ref_set)
        {
            is_first_gps = false;
            ref_set = true;
            // Reconstruct estimator with GPS ref
            pose_estimator = CartesianPose(gps_data, gps_data, vel, acc, imu_data.bearing);
            current_pose = pose_estimator.get_last_cartesian();
            ROS_INFO("acquired?");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int goal_setting()
{
    int move_to_next = 0;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        state::print_goal_setting_status();
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
            return state::SYSTEM_OFF;
        if (dynr::debugging.log_imu_kalman == true)
            return state::BOAT_CONTROLLER;

        if (dynr::current_vessel_task.goal_latitude != 0 && dynr::current_vessel_task.goal_longitude != 0) {
            gps_ref.latitude = dynr::current_vessel_task.goal_latitude;
            gps_ref.longitude = dynr::current_vessel_task.goal_longitude;
            cartesian_ref = pose_estimator.cartesian_pose(gps_ref);
            return state::BOAT_CONTROLLER;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int boat_controller()
{
    int move_to_next = 0;

    ros::Rate loop_rate(10);

    try
    {
        ForceToPWM pwm_converter;
        esc_lib left_esc(17);
        esc_lib right_esc(27);
    }
    catch (...) {
        ROS_ERROR_STREAM("COULD NOT CONSTRUCT ESC OBJECT.");
    }


    while (ros::ok())
    {

        state::print_boat_controller_status();
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
            return state::SYSTEM_OFF;

        if (dynr::debugging.log_gps_raw == true)
            return state::POSE_ESTIMATION;

        if (new_gps)
        {
            current_pose = pose_estimator.cartesian_pose(gps_data);
            new_gps = false;
        }
        else if (new_imu)
        {
            current_pose = pose_estimator.cartesian_pose(imu_data);
            new_imu = false;
        }

        cartesian_error.x = cartesian_ref.position.x - current_pose.position.x;
        cartesian_error.y = cartesian_ref.position.y - current_pose.position.y;
        distance_error = sqrt(pow(cartesian_error.x, 2) + pow(cartesian_error.y, 2));
        bearing_goal = atan2(cartesian_error.y, cartesian_error.x);
        bearing_error = bearing_goal - imu_data.bearing;

        if (bearing_error > M_PI) bearing_error -= 2 * M_PI;
        else if (bearing_error < -M_PI) bearing_error += 2 * M_PI;

        if (distance_error < 2) break;
        else std::cout << "Distance to destiny -> " << distance_error << std::endl;

        // Calculate speeds
        linear_speed = distance_error * dynr::control_gains.linear_gain;
        angular_speed = bearing_error * dynr::control_gains.angular_gain;

        // Calculate forces
        force_drive = linear_speed * dynr::general_config.damping_surge;
        torque_drive = angular_speed * dynr::general_config.damping_yaw;
        force_right = (dynr::general_config.propeller_distance * force_drive - torque_drive) / (2 * dynr::general_config.propeller_distance);
        force_left = force_drive - force_right;

        // Calculate signals
//            pwm_left = pwm_converter.getLeftPWM(force_left);
//            pwm_right = pwm_converter.getRightPWM(force_right);

//            left_esc.setSpeed(pwm_left);
//            right_esc.setSpeed(pwm_right);


        ros::spinOnce();
        loop_rate.sleep();
    }
}

}

#endif //PROJECT_STATE_MACHINE_H
