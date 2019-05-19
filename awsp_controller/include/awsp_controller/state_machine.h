//
// Created by davidm on 4/27/19.
//
#include "awsp_controller/dynamic_parameters.h"
#include "awsp_controller/pd_controller.h"
#include "awsp_pose_estimator/awsp_pose_estimator.h"
#include "awsp_logger/awsp_logger.h"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>

#ifndef PROJECT_STATE_MACHINE_H
#define PROJECT_STATE_MACHINE_H

gps_position gps_data;
imu_data imu_data;
cart_pose cartesian_pose;
//imu_data low_pass_imu_data;
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

struct BoatTestingParams
{
    float force_drive;
    float torque_drive;
    float force_left;
    float force_right;

    float linear_speed;
    float angular_speed;

    int pwm_left;
    int pwm_right;
} boat_testing_params;

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

struct ObstacleData
{
    float front_obstacle_dist;
    bool front_obstacle;
} obstacle_data;

bool is_first_gps = true;
bool ref_set = false;

// Global system level logger, alive until state machine is alive
std::string global_log_file = "boat_global_log.csv";
std::string directory = "/home/ubuntu/awsp_stable_ws/src/awsp_logger/log/";

Logger global_logger(directory);

namespace state {

const int SYSTEM_OFF = 1;
const int POSE_ESTIMATION = 2;
const int GOAL_SETTING = 3;
const int BOAT_CONTROLLER = 4;
const int BOAT_TESTING = 5;

int current_system_state = SYSTEM_OFF;

void log_global()
{

    std::stringstream log_stream;
    std::string ready_to_move_boat;
    if (dynr::current_vessel_task.ready_to_move)
        ready_to_move_boat = "1";
    else
        ready_to_move_boat = "0";

    log_stream << std::fixed << std::setprecision(7)
            << dynr::system_mode.vessel
            << "," << gps_data.latitude
            << "," << gps_data.longitude
            << "," << dynr::current_vessel_task.goal_latitude
            << "," << dynr::current_vessel_task.goal_longitude
            << "," << imu_data.acceleration.x
            << "," << imu_data.acceleration.y
            << "," << imu_data.yaw_vel
            << "," << dynr::control_gains.use_imu_bearing
            << "," << boat_control_params.pwm_right
            << "," << boat_control_params.pwm_left
            << "," << boat_control_params.torque_drive
            << "," << boat_control_params.force_right
            << "," << boat_control_params.force_left
            << "," << ready_to_move_boat
            << "," << current_pose.position.x
            << "," << current_pose.position.y
            << "," << current_pose.bearing
            << "," << cartesian_ref.position.x
            << "," << cartesian_ref.position.y
            << "," << boat_control_params.cartesian_error.x
            << "," << boat_control_params.cartesian_error.y
            << "," << boat_control_params.distance_error
            << "," << boat_control_params.bearing_error
            << "," << boat_control_params.bearing_goal
            << "," << boat_control_params.linear_speed
            << "," << dynr::control_gains.p_linear_gain
		    << "," << dynr::control_gains.d_linear_gain
            << "," << dynr::control_gains.p_angular_gain
		    << "," << dynr::control_gains.d_angular_gain
            << "," << dynr::control_gains.use_fault_detection
            << "," << dynr::current_vessel_task.distance_error_tol
            << "," << gps_data.speed
            << "," << gps_data.true_course;

    global_logger.additional_logger(log_stream.str(), global_log_file);
}

void print_system_off_status()
{
    ROS_WARN_STREAM( "[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM( "[SYSTEM IS IN STATE     ] 1 - SYSTEM_OFF");
    ROS_DEBUG_STREAM("[P LINEAR GAIN          ] " << dynr::control_gains.p_linear_gain);
	ROS_DEBUG_STREAM("[D LINEAR GAIN          ] " << dynr::control_gains.d_linear_gain);
    ROS_DEBUG_STREAM("[P ANGULAR GAIN         ] " << dynr::control_gains.p_angular_gain);
	ROS_DEBUG_STREAM("[D ANGULAR GAIN         ] " << dynr::control_gains.d_angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);
//    ROS_DEBUG_STREAM();
    ROS_DEBUG("================================================");
}

void print_pose_estimation_status(gps_position gps_data, cart_pose current_pose)
{
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

	ROS_DEBUG_STREAM("[P LINEAR GAIN          ] " << dynr::control_gains.p_linear_gain);
	ROS_DEBUG_STREAM("[D LINEAR GAIN          ] " << dynr::control_gains.d_linear_gain);
	ROS_DEBUG_STREAM("[P ANGULAR GAIN         ] " << dynr::control_gains.p_angular_gain);
	ROS_DEBUG_STREAM("[D ANGULAR GAIN         ] " << dynr::control_gains.d_angular_gain);
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
    ROS_DEBUG_STREAM("[USE IMU BEARING        ] " << dynr::control_gains.use_imu_bearing);

    ROS_DEBUG_STREAM("[CURRENT LATITUDE       ] " << gps_data.latitude);
    ROS_DEBUG_STREAM("[CURRENT LONGITUDE      ] " << gps_data.longitude);
    ROS_DEBUG_STREAM("[CURRENT CART X         ] " << current_pose.position.x);
    ROS_DEBUG_STREAM("[CURRENT CART Y         ] " << current_pose.position.y);
    ROS_DEBUG_STREAM("[CURRENT BEARING        ] " << current_pose.bearing);

	ROS_DEBUG_STREAM("[P LINEAR GAIN          ] " << dynr::control_gains.p_linear_gain);
	ROS_DEBUG_STREAM("[D LINEAR GAIN          ] " << dynr::control_gains.d_linear_gain);
	ROS_DEBUG_STREAM("[P ANGULAR GAIN         ] " << dynr::control_gains.p_angular_gain);
	ROS_DEBUG_STREAM("[D ANGULAR GAIN         ] " << dynr::control_gains.d_angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);

    ROS_DEBUG("================================================");
}

void print_boat_controller_status(BoatControlParams boat_control_params)
{
    ROS_WARN_STREAM( "[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM( "[SYSTEM IS IN STATE     ] 4 - BOAT_CONTROLLER");
    ROS_DEBUG_STREAM("[CURRENT GOAL LATITUDE  ] " << dynr::current_vessel_task.goal_latitude);
    ROS_DEBUG_STREAM("[CURRENT GOAL LONGITUDE ] " << dynr::current_vessel_task.goal_longitude);

    ROS_DEBUG_STREAM("[DISTANCE ERROR         ] " << boat_control_params.distance_error);
    ROS_DEBUG_STREAM("[DISTANCE ERROR TOL     ] " << dynr::current_vessel_task.distance_error_tol);
    ROS_DEBUG_STREAM("[USE IMU BEARING        ] " << dynr::control_gains.use_imu_bearing);
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

	ROS_DEBUG_STREAM("[P LINEAR GAIN          ] " << dynr::control_gains.p_linear_gain);
	ROS_DEBUG_STREAM("[D LINEAR GAIN          ] " << dynr::control_gains.d_linear_gain);
	ROS_DEBUG_STREAM("[P ANGULAR GAIN         ] " << dynr::control_gains.p_angular_gain);
	ROS_DEBUG_STREAM("[D ANGULAR GAIN         ] " << dynr::control_gains.d_angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);

    ROS_DEBUG("================================================");
}

void print_boat_testing_status(BoatTestingParams boat_testing_params)
{
    ROS_WARN_STREAM( "[SYSTEM IS CURRENTLY    ] " << dynr::system_mode.vessel);
    ROS_WARN_STREAM( "[SYSTEM IS IN STATE     ] 5 - BOAT_TESTING");
    ROS_WARN_STREAM( "[READY TO TEST          ] " << dynr::boat_testing_config.ready_to_test);

    ROS_DEBUG_STREAM("[FORCE DRIVE            ] " << boat_testing_params.force_drive);
    ROS_DEBUG_STREAM("[FORCE RIGHT            ] " << boat_testing_params.force_right);
    ROS_DEBUG_STREAM("[FORCE LEFT             ] " << boat_testing_params.force_left);
    ROS_DEBUG_STREAM("[TORQUE DRIVE           ] " << boat_testing_params.torque_drive);
    ROS_DEBUG_STREAM("[LINEAR VELOCITY        ] " << boat_testing_params.linear_speed);
    ROS_DEBUG_STREAM("[ANGULAR VELOCITY       ] " << boat_testing_params.angular_speed);
    ROS_DEBUG_STREAM("[PWM LEFT               ] " << boat_testing_params.pwm_left);
    ROS_DEBUG_STREAM("[PWM RIGHT              ] " << boat_testing_params.pwm_right);

	ROS_DEBUG_STREAM("[P LINEAR GAIN          ] " << dynr::control_gains.p_linear_gain);
	ROS_DEBUG_STREAM("[D LINEAR GAIN          ] " << dynr::control_gains.d_linear_gain);
	ROS_DEBUG_STREAM("[P ANGULAR GAIN         ] " << dynr::control_gains.p_angular_gain);
	ROS_DEBUG_STREAM("[D ANGULAR GAIN         ] " << dynr::control_gains.d_angular_gain);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);

    ROS_DEBUG("================================================");
}

std::vector<std::vector<float>> load_gps_waypoints()
{
    std::string file_name = "/home/ubuntu/awsp_stable_ws/src/awsp_controller/waypoints/waypoints.csv";
    std::fstream gps_file(file_name);
    std::string line = "";
    std::vector<std::vector<float> > data;

    while (getline(gps_file, line))                   // read a whole line of the file
    {
        std::stringstream ss(line);                     // put it in a stringstream (internal stream)
        std::vector<float> row;
        std::string data_s;
        while (getline(ss, data_s, ',' ))           // read (string) items up to a comma
        {
            row.push_back(stod(data_s));            // use stod() to convert to double; put in row vector
        }
        if (row.size() > 0)
            data.push_back(row);    // add non-empty rows to matrix
    }
    return data;
}

int evaluate_system_mode_status()
{
    if (dynr::system_mode.vessel == OFF)
    {
        return SYSTEM_OFF;
    }

    return -1;
}

int system_off()
{
    if (dynr::system_mode.vessel == OFF)
    {
        dynr::current_vessel_task.goal_latitude = 0;
        dynr::current_vessel_task.goal_longitude = 0;

        ros::Rate loop_rate(10);

        while (ros::ok()) {
            // print status while we are waiting for the initial startup
            log_global();
            state::print_system_off_status();

            if (dynr::system_mode.vessel == ON) {
                ROS_WARN("THE SYSTEM IS NOW TURNING ON.");
                return POSE_ESTIMATION;
            } else if (dynr::system_mode.vessel == TESTING) {
                ROS_WARN("THE SYSTEM IS GOING INTO TESTING MODE");
                return BOAT_TESTING;
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
        log_global();
        if (evaluate_system_mode_status() == state::SYSTEM_OFF)
        {
            return state::SYSTEM_OFF;
        }

        if (dynr::state_bypass.bypass_2_3 == true)
            return state::GOAL_SETTING;



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
        log_global();
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
            return state::SYSTEM_OFF;

        if (dynr::current_vessel_task.goal_latitude != 0
            && dynr::current_vessel_task.goal_longitude != 0
                && dynr::current_vessel_task.ready_to_move == true
                    && !dynr::current_vessel_task.use_gps_waypoints)
        {
            gps_ref.latitude = dynr::current_vessel_task.goal_latitude;
            gps_ref.longitude = dynr::current_vessel_task.goal_longitude;
            return state::BOAT_CONTROLLER;
        }
        else if (dynr::current_vessel_task.use_gps_waypoints == true)
        {
            std::vector<std::vector<float>> gps_points = load_gps_waypoints();
//            std::cout << gps_points.size() << std::endl;
//            for (int i = 0; i < gps_points.size(); i++)
//            {
//                std::cout << gps_points[i][0] << " -- " << gps_points[i][1] << std::endl;
////                for (int j = 0; j < gps_points[i].size(); j++)
////                {
////                    ROS_ERROR("PRINT SOMETHING YOU FUCK222");
////
////                }
//            }
        	return state::BOAT_CONTROLLER;
        }

        if (dynr::state_bypass.bypass_3_4 == true)
        	return state::BOAT_CONTROLLER;
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

    bool left_esc_alive = left_esc.setup();
    bool right_esc_alive = right_esc.setup();

    if (!left_esc_alive || !right_esc_alive)
    {
        ROS_ERROR("ESC LIB FAILED.");
    }
    ros::Duration(3).sleep();

    PDController surge_pd_ctrl(0.1, 15, -15);
    PDController yaw_pd_ctrl(0.1, 15, -15);

    while (ros::ok())
    {
        log_global();
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
        {
            left_esc.end();
            right_esc.end();
            return state::SYSTEM_OFF;
        }

        // bypass to next state
        if (dynr::state_bypass.bypass_4_2 == true)
        {
            left_esc.end();
            right_esc.end();
            return state::POSE_ESTIMATION;
        }

        if (obstacle_data.front_obstacle && dynr::control_gains.use_obstacle_detector)
        {
            left_esc.end();
            right_esc.end();
            ROS_WARN("OBSTACLE DETECTED, STOPPING.");
            return state::POSE_ESTIMATION;
        }

        boat_control_params.cartesian_error.x = cartesian_pose.goal_x - cartesian_pose.position.x;
        boat_control_params.cartesian_error.y = cartesian_pose.goal_y - cartesian_pose.position.y;
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

        if(!dynr::control_gains.use_imu_bearing)
        {
            boat_control_params.bearing_error = 0;
        }

        if (boat_control_params.distance_error < dynr::current_vessel_task.distance_error_tol)
        {
            return state::POSE_ESTIMATION;
        }

        // Calculate forces
        boat_control_params.force_drive = surge_pd_ctrl.get_force(boat_control_params.distance_error,
                dynr::control_gains.p_linear_gain, dynr::control_gains.d_linear_gain);
        boat_control_params.torque_drive = yaw_pd_ctrl.get_force(boat_control_params.bearing_error,
                dynr::control_gains.p_angular_gain, dynr::control_gains.d_angular_gain);
        boat_control_params.force_right = (dynr::general_config.propeller_distance * boat_control_params.force_drive
                - boat_control_params.torque_drive) / (2 * dynr::general_config.propeller_distance);
        boat_control_params.force_left = boat_control_params.force_drive - boat_control_params.force_right;

        boat_control_params.force_left = boat_control_params.force_left * dynr::control_gains.left_motor_force_gain;
        boat_control_params.force_right = boat_control_params.force_right * dynr::control_gains.right_motor_force_gain;

//        Calculate signals
        boat_control_params.pwm_left = pwm_converter.getLeftPWM(boat_control_params.force_left);
        boat_control_params.pwm_right = pwm_converter.getRightPWM(boat_control_params.force_right);

        left_esc.setSpeed(boat_control_params.pwm_left);
        right_esc.setSpeed(boat_control_params.pwm_right);

        state::print_boat_controller_status(boat_control_params);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int boat_testing()
{
    ros::Rate loop_rate(10);

    std::string testing_file = "boat_testing.csv";
	std::string sensor_testing_data;
    std::string directory = "/home/ubuntu/awsp_stable_ws/src/awsp_logger/log/";

	Logger logger(directory);

    ForceToPWM pwm_converter;
    esc_lib left_esc(17);
    esc_lib right_esc(27);

    bool left_esc_alive = left_esc.setup();
    bool right_esc_alive = right_esc.setup();

    if (!left_esc_alive || !right_esc_alive)
    {
        ROS_ERROR("ESC LIB FAILED.");
    }
    ros::Duration(3).sleep();
    while(ros::ok())
    {
        if (state::evaluate_system_mode_status() == state::SYSTEM_OFF)
        {
            left_esc.end();
            right_esc.end();
            return state::SYSTEM_OFF;
        }

        if (dynr::boat_testing_config.log_sensors_testing == true)
        {
            float left_temp_force;
            left_temp_force = dynr::boat_testing_config.left_motor_force;
            float right_temp_force;
            right_temp_force = dynr::boat_testing_config.right_motor_force;
            std::stringstream stream_out;
        	int ready_to_test_out;
        	if (dynr::boat_testing_config.ready_to_test)
		        ready_to_test_out = 1;
            else
<<<<<<< Updated upstream
		        ready_to_test_out = "0";
=======
		        ready_to_test_out = 0;
//
            if (ready_to_test_out == 0)
            {
                left_temp_force = 0;
                right_temp_force = 0;
            }
>>>>>>> Stashed changes

            stream_out << std::fixed << std::setprecision(7)
	                << gps_data.latitude
	                << "," << gps_data.longitude
	                << "," << imu_data.acceleration.x
	                << "," << imu_data.acceleration.y
	                << "," << imu_data.yaw_vel
	                << "," << boat_testing_params.pwm_right
		            << "," << boat_testing_params.pwm_left
		            << "," << right_temp_force
		            << "," << left_temp_force
		            << "," << ready_to_test_out
                    << "," << gps_data.speed
                    << "," << gps_data.true_course
                    << "," << obstacle_data.front_obstacle_dist
                    << "," << obstacle_data.front_obstacle;
            logger.additional_logger(stream_out.str(), testing_file);
        }

        if (obstacle_data.front_obstacle && dynr::control_gains.use_obstacle_detector)
        {
//            left_esc.end();
//            right_esc.end();
            boat_testing_params.force_right = -3;
            ROS_WARN("OBSTACLE DETECTED, STOPPING.");
//            return state::SYSTEM_OFF;
        }

	    boat_testing_params.force_right = dynr::boat_testing_config.right_motor_force;
	    boat_testing_params.force_left = dynr::boat_testing_config.left_motor_force;

        if (dynr::boat_testing_config.forward_force == false)
        {
            boat_testing_params.force_right = -14;
            boat_testing_params.force_left = -14;
        }

// 	    if (dynr::boat_testing_config.max_force_right_motor == true
// 	        && dynr::boat_testing_config.forward_force == true)
// 	    {
// 		    boat_testing_params.force_right = 15;
// //                boat_testing_params.force_left = 0;
// 	    }
// 	    else if (dynr::boat_testing_config.max_force_left_motor == true
// 	             && dynr::boat_testing_config.forward_force == true)
// 	    {
// //                boat_testing_params.force_right = 0;
// 		    boat_testing_params.force_left = 15;
// 	    }
// 	    else if (dynr::boat_testing_config.max_force_right_motor == true
// 	             && dynr::boat_testing_config.forward_force == false)
// 	    {
// 		    boat_testing_params.force_right = -15;
// //                boat_testing_params.force_left = 0;
// 	    }
// 	    else if (dynr::boat_testing_config.max_force_left_motor == true
// 	             && dynr::boat_testing_config.forward_force == false)
// 	    {
// //                boat_testing_params.force_right = 0;
// 		    boat_testing_params.force_left = -15;
// 	    }

	    boat_testing_params.pwm_right = pwm_converter.getRightPWM(boat_testing_params.force_right);
	    boat_testing_params.pwm_left = pwm_converter.getLeftPWM(boat_testing_params.force_left);

        if (dynr::boat_testing_config.ready_to_test == true &&
                dynr::boat_testing_config.use_pwm == false)
        {
	        right_esc.setSpeed(boat_testing_params.pwm_right);
	        left_esc.setSpeed(boat_testing_params.pwm_left);
        }
        else if (dynr::boat_testing_config.ready_to_test == true &&
                dynr::boat_testing_config.use_pwm == true)
        {
            boat_testing_params.pwm_right = dynr::boat_testing_config.right_motor_pwm;
            boat_testing_params.pwm_left = dynr::boat_testing_config.left_motor_pwm;
            right_esc.setSpeed(boat_testing_params.pwm_right);
            left_esc.setSpeed(boat_testing_params.pwm_left);
        }
        else
        {
	        right_esc.setSpeed(1500);
	        left_esc.setSpeed(1500);
        }

        state::print_boat_testing_status(boat_testing_params);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

}

#endif //PROJECT_STATE_MACHINE_H
