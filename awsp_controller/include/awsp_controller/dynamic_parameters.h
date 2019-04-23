//
// Created by davidm on 4/21/19.
//

#ifndef PROJECT_DYNAMIC_PARAMETERS_H
#define PROJECT_DYNAMIC_PARAMETERS_H

#define ON 1
#define OFF 0

// Needs to have default values to change from

namespace dynr
{
namespace LEVEL {
const int GAINS = 0;
const int DYNAMIC_MODEL = 1;
const int SYSTEM_MODE = 2;
const int LLA_GOAL_POINTS = 3;
const int CROSS_GROUP_LOG = 4;
const int DEBUGGING = 5;
}

struct ControlGains {
    float linear_gain = 0.0269;
    float angular_gain = 0.167;
    bool log_control_system_config = false;
    bool use_fault_detection = true;
} control_gains;

struct SystemMode {
    bool vessel = OFF;
    bool evaluate_system_mode();
} system_mode;

struct CurrentVesselTask {
    float goal_latitude;
    float goal_longitude;
} current_vessel_task;

struct GeneralConfig {
    float damping_surge;
    float damping_yaw;
    float propeller_distance;
    bool log_general_config = false;
} general_config;

struct Debugging {
    bool log_imu_raw = false;
    bool log_imu_kalman = false;
    bool log_gps_raw = false;
    bool log_gps_kalman = false;
}debugging;

void print_system_status()
{
    ROS_DEBUG_STREAM("[CURRENT GOAL LATITUDE  ] " << dynr::current_vessel_task.goal_latitude);
    ROS_DEBUG_STREAM("[CURRENT GOAL LONGITUDE ] " << dynr::current_vessel_task.goal_longitude);
    ROS_DEBUG_STREAM("[SYSTEM MODE IS         ] " << dynr::system_mode.vessel);
    ROS_DEBUG_STREAM("[LINEAR GAIN            ] " << dynr::control_gains.linear_gain);
    ROS_DEBUG_STREAM("[ANGULAR GAIN           ] " << dynr::control_gains.angular_gain);
    ROS_DEBUG_STREAM("[USE FAULT DETECTION    ] " << dynr::control_gains.use_fault_detection);
    ROS_DEBUG_STREAM("[LOG CONTROL SYSTEM     ] " << dynr::control_gains.log_control_system_config);
//    ROS_DEBUG_STREAM();
    ROS_DEBUG("================================================");
}


}

bool dynr::SystemMode::evaluate_system_mode()
{
    if (dynr::system_mode.vessel == OFF) {
        ROS_WARN("The system is now TURNING OFF.");
        ROS_INFO("The system is currently OFF, waiting for initialization.");
        while (ros::ok()) {

            if (dynr::system_mode.vessel == ON) {
                ROS_WARN("The system is now TURNING ON.");
                break;
            }
            ros::spinOnce();
        }
    }
    else if (dynr::system_mode.vessel == ON)
    {
        return true;
    }
}

#endif //PROJECT_DYNAMIC_PARAMETERS_H
