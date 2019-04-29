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
} system_mode;

struct CurrentVesselTask {
    float goal_latitude;
    float goal_longitude;
    bool ready_to_move;
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


}


#endif //PROJECT_DYNAMIC_PARAMETERS_H
