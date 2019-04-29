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

struct BoatControlParams
{
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
} boat_control_params;


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

void print_pose_estimation_status(gps_position gps_data, cart_pose current_pose) {
    ROS_WARN_STREAM( "[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM( "[SYSTEM IS IN STATE     ] 2 - POSE_ESTIMATION");

    ROS_DEBUG_STREAM("[GPS FIX STATUS         ] " << gps_data.fix);
    ROS_DEBUG_STREAM("[GOAL LATITUDE          ] " << dynr::current_vessel_task.goal_latitude);
    ROS_DEBUG_STREAM("[GOAL LONGITUDE         ] " << dynr::current_vessel_task.goal_longitude);

    ROS_DEBUG_STREAM("[CURRENT LATITUDE       ] " << gps_data.latitude);
    ROS_DEBUG_STREAM("[CURRENT LONGITUDE      ] " << gps_data.longitude);
    ROS_DEBUG_STREAM("[CURRENT CART X         ] " << current_pose.position.x);
    ROS_DEBUG_STREAM("[CURRENT CART Y         ] " << current_pose.position.y);
    ROS_DEBUG_STREAM("[CURRENT BEARING        ] " << current_pose.bearing);

    ROS_DEBUG_STREAM("[LINEAR GAIN            ] " << dynr::control_gains.linear_gain);
    ROS_DEBUG_STREAM("[ANGULAR GAIN           ] " << dynr::control_gains.angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);

    ROS_DEBUG("================================================");
}

void print_goal_setting_status(gps_position gps_data, cart_pose current_pose)
{
    ROS_WARN_STREAM( "[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM( "[SYSTEM IS IN STATE     ] 3 - GOAL_SETTING");

    ROS_DEBUG_STREAM("[GPS FIX STATUS         ] " << gps_data.fix);
    ROS_DEBUG_STREAM("[GOAL LATITUDE          ] " << dynr::current_vessel_task.goal_latitude);
    ROS_DEBUG_STREAM("[GOAL LONGITUDE         ] " << dynr::current_vessel_task.goal_longitude);
    ROS_DEBUG_STREAM("[DISTANCE ERROR TOL     ] " << dynr::current_vessel_task.distance_error_tol);

    ROS_DEBUG_STREAM("[CURRENT LATITUDE       ] " << gps_data.latitude);
    ROS_DEBUG_STREAM("[CURRENT LONGITUDE      ] " << gps_data.longitude);
    ROS_DEBUG_STREAM("[CURRENT CART X         ] " << current_pose.position.x);
    ROS_DEBUG_STREAM("[CURRENT CART Y         ] " << current_pose.position.y);
    ROS_DEBUG_STREAM("[CURRENT BEARING        ] " << current_pose.bearing);

    ROS_DEBUG_STREAM("[LINEAR GAIN            ] " << dynr::control_gains.linear_gain);
    ROS_DEBUG_STREAM("[ANGULAR GAIN           ] " << dynr::control_gains.angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);

    ROS_DEBUG("================================================");
}

void print_boat_controller_status(BoatControlParams boat_control_params)
{
    ROS_WARN_STREAM("[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM("[SYSTEM IS IN STATE     ] 4 - BOAT_CONTROLLER");
    ROS_DEBUG_STREAM("[CURRENT GOAL LATITUDE  ] " << dynr::current_vessel_task.goal_latitude);
    ROS_DEBUG_STREAM("[CURRENT GOAL LONGITUDE ] " << dynr::current_vessel_task.goal_longitude);

    ROS_DEBUG_STREAM("[DISTANCE ERROR         ] " << boat_control_params.distance_error);
    ROS_DEBUG_STREAM("[DISTANCE ERROR TOL     ] " << dynr::current_vessel_task.distance_error_tol);
    ROS_DEBUG_STREAM("[BEARING GOAL           ] " << boat_control_params.bearing_goal);
    ROS_DEBUG_STREAM("[BEARING ERROR          ] " << boat_control_params.bearing_error);
    ROS_DEBUG_STREAM("[FORCE DRIVE            ] " << boat_control_params.force_drive);
    ROS_DEBUG_STREAM("[FORCE RIGHT            ] " << boat_control_params.force_right);
    ROS_DEBUG_STREAM("[FORCE LEFT             ] " << boat_control_params.force_left);
    ROS_DEBUG_STREAM("[TORQUE DRIVE           ] " << boat_control_params.torque_drive);
    ROS_DEBUG_STREAM("[LINEAR VELOCITY        ] " << boat_control_params.linear_speed);
    ROS_DEBUG_STREAM("[ANGULAR VELOCITY       ] " << boat_control_params.angular_speed);
    ROS_DEBUG_STREAM("[PWM LEFT               ] " << boat_control_params.pwm_left);
    ROS_DEBUG_STREAM("[PWM RIGHT              ] " << boat_control_params.pwm_right);

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

        dynr::current_vessel_task.goal_latitude = 0;
        dynr::current_vessel_task.goal_longitude = 0;

        ros::Rate loop_rate(10);

        while (ros::ok()) {
            // print status while we are waiting for the initial startup

            state::print_system_off_status();

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
        }

        state::print_pose_estimation_status(gps_data, current_pose);
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

        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
            return state::SYSTEM_OFF;
//        if (dynr::current_vessel_task.ready_to_move == true)
//            return state::BOAT_CONTROLLER;

        if (dynr::current_vessel_task.goal_latitude != 0
            && dynr::current_vessel_task.goal_longitude != 0
                && dynr::current_vessel_task.ready_to_move == true)
        {
            gps_ref.latitude = dynr::current_vessel_task.goal_latitude;
            gps_ref.longitude = dynr::current_vessel_task.goal_longitude;
            cartesian_ref = pose_estimator.cartesian_pose(gps_ref);
            return state::BOAT_CONTROLLER;
        }
        state::print_goal_setting_status(gps_data, current_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int boat_controller()
{
    int move_to_next = 0;

    ros::Rate loop_rate(10);

    ForceToPWM pwm_converter;
    esc_lib left_esc(17);
    esc_lib right_esc(27);

    if (!left_esc.setup() && !right_esc.setup())
    {
        ROS_ERROR("ESC LIB FAILED.");
        ros::Duration(3).sleep();
    }

    while (ros::ok())
    {

//        state::print_boat_controller_status();
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

        boat_control_params.cartesian_error.x = cartesian_ref.position.x - current_pose.position.x;
        boat_control_params.cartesian_error.y = cartesian_ref.position.y - current_pose.position.y;
        boat_control_params.distance_error = sqrt(pow(boat_control_params.cartesian_error.x, 2) + pow(boat_control_params.cartesian_error.y, 2));
        boat_control_params.bearing_goal = atan2(boat_control_params.cartesian_error.y, boat_control_params.cartesian_error.x);
        boat_control_params.bearing_error = boat_control_params.bearing_goal - imu_data.bearing;

        if (boat_control_params.bearing_error > M_PI)
        {
            boat_control_params.bearing_error -= 2 * M_PI;
        }
        else if (boat_control_params.bearing_error < -M_PI)
        {
            boat_control_params.bearing_error += 2 * M_PI;
        }

        if (boat_control_params.distance_error < dynr::current_vessel_task.distance_error_tol)
        {
            return state::POSE_ESTIMATION;
        }

        // Calculate speeds
        boat_control_params.linear_speed = boat_control_params.distance_error * dynr::control_gains.linear_gain;
        boat_control_params.angular_speed = boat_control_params.bearing_error * dynr::control_gains.angular_gain;

        // Calculate forces
        boat_control_params.force_drive = boat_control_params.linear_speed * dynr::general_config.damping_surge;
        boat_control_params.torque_drive = boat_control_params.angular_speed * dynr::general_config.damping_yaw;
        boat_control_params.force_right = (dynr::general_config.propeller_distance * boat_control_params.force_drive - boat_control_params.torque_drive) / (2 * dynr::general_config.propeller_distance);
        boat_control_params.force_left = boat_control_params.force_drive - boat_control_params.force_right;

//        Calculate signals
        boat_control_params.pwm_left = pwm_converter.getLeftPWM(boat_control_params.force_left);
        boat_control_params.pwm_right = pwm_converter.getRightPWM(boat_control_params.force_right);

//        left_esc.setSpeed(boat_control_params.pwm_left);
//        right_esc.setSpeed(boat_control_params.pwm_right);

        state::print_boat_controller_status(boat_control_params);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

}

#endif //PROJECT_STATE_MACHINE_H
