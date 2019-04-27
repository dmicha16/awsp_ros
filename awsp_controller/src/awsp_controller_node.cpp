#include <iostream>
#include <ros/console.h>
#include <math.h>
#include "ros/ros.h"
#include "awsp_controller/force_to_pwm.h"
#include "awsp_controller/esc_lib.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"
#include "awsp_gy_88_interface/gy_88_lib.h"
#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"

#include "awsp_controller/state_machine.h"

#include <dynamic_reconfigure/server.h>
#include <awsp_controller/ParametersConfig.h>

void callback(awsp_controller::ParametersConfig &config, uint32_t level) {

    switch (level) {
        case dynr::LEVEL::GAINS:
            dynr::control_gains.linear_gain = config.linear_gain;
            dynr::control_gains.angular_gain = config.angular_gain;
            dynr::control_gains.use_fault_detection = config.use_fault_detection;
            break;
        case dynr::LEVEL::DYNAMIC_MODEL:
            dynr::general_config.damping_surge = config.damping_surge;
            dynr::general_config.damping_yaw = config.damping_yaw;
            dynr::general_config.propeller_distance = config.propeller_distance;
            break;
        case dynr::LEVEL::SYSTEM_MODE:
            dynr::system_mode.vessel = config.system_mode;
            break;
        case dynr::LEVEL::LLA_GOAL_POINTS:
            dynr::current_vessel_task.goal_latitude = config.goal_latitude;
            dynr::current_vessel_task.goal_longitude = config.goal_longitude;
            break;
        case dynr::LEVEL::CROSS_GROUP_LOG:
            dynr::general_config.log_general_config = config.log_general_config;
            dynr::control_gains.log_control_system_config = config.log_control_system_config;
            break;
        case dynr::LEVEL::DEBUGGING:
            dynr::debugging.log_gps_raw = config.log_gps_raw;
            dynr::debugging.log_gps_kalman = config.log_gps_kalman;
            dynr::debugging.log_imu_raw = config.log_imu_raw;
            dynr::debugging.log_imu_kalman = config.log_imu_kalman;
            break;
    }
}

void gnss_data_callback(const awsp_msgs::GnssData::ConstPtr& gnss_msg)
{
    gps_data.latitude = gnss_msg->latitude;
    gps_data.longitude = gnss_msg->longitude;
    gps_data.timestamp = gnss_msg->timestamp;
    new_gps = true;
}

void imu_data_callback(const awsp_msgs::Gy88Data::ConstPtr& imu_msg)
{
    // imu_data.acceleration.x = imu_msg->si_accel_x;
    // imu_data.acceleration.y = -imu_msg->si_accel_y;
    imu_data.acceleration.x = floorf(imu_msg->si_accel_x * 100) / 100;
    imu_data.acceleration.y = -floorf(imu_msg->si_accel_y * 100) / 100;
    imu_data.yaw_vel = -imu_msg->gyro_z;
    imu_data.bearing = imu_msg->compass_angle;
    imu_data.timestamp = imu_msg->timestamp;
    new_imu = true;
}

int main(int argc, char **argv)
{
    // ****************** Node Initialization ******************

    ROS_DEBUG("===== INITIALIZING CONTROLLER NODE =====");

    ros::init(argc, argv, "awsp_controller_node");
    ros::NodeHandle n;
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Rate loop_rate(10);

    dynamic_reconfigure::Server<awsp_controller::ParametersConfig> server;
    dynamic_reconfigure::Server<awsp_controller::ParametersConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while (ros::ok())
    {
        switch (state::current_system_state)
        {
            case state::SYSTEM_OFF:
                state::system_off();
                state::current_system_state = state::POSE_ESTIMATION;
                break;
            case state::POSE_ESTIMATION:
                state::current_system_state = state::pose_estimation();
                break;
            case state::GOAL_SETTING:
                state::current_system_state = state::goal_setting();
                break;
            case state::BOAT_CONTROLLER:
                state::current_system_state = state::boat_controller();
                break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
