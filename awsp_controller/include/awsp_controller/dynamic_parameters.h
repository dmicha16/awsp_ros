//
// Created by davidm on 4/21/19.
//

#ifndef PROJECT_DYNAMIC_PARAMETERS_H
#define PROJECT_DYNAMIC_PARAMETERS_H

#define OFF     1
#define ON      0
#define TESTING 2

// Needs to have default values to change from

namespace dynr
{
namespace LEVEL
{
const int GAINS              = 0;
const int DYNAMIC_MODEL      = 1;
const int SYSTEM_MODE        = 2;
const int LLA_GOAL_POINTS    = 3;
const int CROSS_GROUP_LOG    = 4;
const int DEBUGGING          = 5;
const int LOW_PASS_FILTERING = 6;
const int BOAT_TESTING       = 7;
const int STATE_BYPASS       = 8;
}

namespace FILTER
{
const int SMA = 0;
const int EMA = 1;
const int NONE = 2;

}

struct ControlGains
{
    float p_linear_gain = 0.35014830776512;
	float d_linear_gain = 3.41234381251693;
    float p_angular_gain = 1.37254843957504;
	float d_angular_gain = 3.22145409227058;
	float left_motor_force_gain = 1;
	float right_motor_force_gain = 1;
    bool log_control_system_config = false;
    bool use_fault_detection = true;
    bool use_imu_bearing = true;
    bool use_obstacle_detector = true;
} control_gains;

struct SystemMode
{
    int vessel = OFF;
} system_mode;

struct CurrentVesselTask
{
    float goal_latitude;
    float goal_longitude;
    bool ready_to_move;
    int distance_error_tol;
    bool log_goal_points = true;
    bool use_gps_waypoints = false;

} current_vessel_task;

struct GeneralConfig
{
    float damping_surge;
    float damping_yaw;
    float propeller_distance;
    bool log_general_config = false;
} general_config;

struct Debugging
{
    bool log_imu_raw = false;
    bool log_imu_kalman = false;
    bool log_gps_raw = false;
    bool log_gps_kalman = false;
    bool log_state_machine = false;
}debugging;

struct BoatTestingConfig
{
    bool ready_to_test = false;
    int left_motor_force = 0;
    int right_motor_force = 0;
	int left_motor_pwm = 1500;
	int right_motor_pwm = 1500;
    bool max_force_right_motor = false;
    bool max_force_left_motor = false;
    bool forward_force = true;
    bool log_sensors_testing = true;
    bool use_pwm = false;
} boat_testing_config;

struct StateBypass
{
	bool bypass_2_3 = false;
	bool bypass_3_4 = false;
	bool bypass_4_2 = false;
} state_bypass;

}


#endif //PROJECT_DYNAMIC_PARAMETERS_H
