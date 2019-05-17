//
// Created by davidm on 5/16/19.
//

#ifndef PROJECT_POSE_PARAMETERS_H
#define PROJECT_POSE_PARAMETERS_H

namespace dynr_p
{

namespace FILTER
{
const int SMA = 0;
const int EMA = 1;
const int NONE = 2;

}

struct LowPassFilteringConfig
{
    int filtering_mode = FILTER::NONE;
    bool imu_acc = false;
    bool imu_gyro = false;
    int window_size = 100;
    double alpha_weight = 1;
} low_pass_filtering_config;

}
#endif //PROJECT_POSE_PARAMETERS_H
