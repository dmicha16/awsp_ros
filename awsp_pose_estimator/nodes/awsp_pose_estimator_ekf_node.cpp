#include <chrono>
#include <cstdlib>
#include <fstream>
#include <math.h>
#include <string>
#include <ros/console.h>
#include "ros/ros.h"
#include "awsp_pose_estimator/awsp_pose_estimator.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"
#include "awsp_gy_88_interface/gy_88_lib.h"
#include "awsp_sensor_filter_kit/awsp_sensor_filter_kit_lib.h"
#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/SensorKitData.h"
#include "awsp_msgs/GoalCoordinates.h"
#include "awsp_msgs/CartesianPose.h"
#include "awsp_pose_estimator/pose_parameters.h"

#include <dynamic_reconfigure/server.h>
#include <awsp_pose_estimator/PoseParametersConfig.h>

gps_position gps_data;
gps_position goal_gps_data;
imu_data imu_data, filtered_imu;
cart_pose cartesian_ref;
bool new_imu = false;
bool new_gps = false;
bool new_goal = false;
FilterKit filter_kit(6);

void dynr_p_callback(awsp_pose_estimator::PoseParametersConfig &config, uint32_t level)
{
    dynr_p::low_pass_filtering_config.filtering_mode = config.low_pass_filtering_mode;
    dynr_p::low_pass_filtering_config.imu_acc = config.low_pass_imu_acc;
    dynr_p::low_pass_filtering_config.imu_gyro = config.low_pass_imu_gyro;
    dynr_p::low_pass_filtering_config.window_size = config.window_size;
    dynr_p::low_pass_filtering_config.alpha_weight = config.alpha_weight;
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
    imu_data.acceleration.x = floorf(imu_msg->si_accel_x * 100) / 100; // Put 10 for one decimal, 100 for 2, 1000 for 3, etc.
    imu_data.acceleration.y = -floorf(imu_msg->si_accel_y * 100) / 100;
    imu_data.accel_z = imu_msg->si_accel_z;
    imu_data.vel_x = imu_msg->vel_x;
    imu_data.vel_y = imu_msg->vel_y;
    imu_data.vel_z = imu_msg->vel_z;

    imu_data.gyro.x = imu_msg->gyro_x;
    imu_data.gyro.y = imu_msg->gyro_y;
    imu_data.gyro.z = imu_msg->gyro_z;
    imu_data.yaw_vel = imu_msg->gyro_z;
    imu_data.bearing = imu_msg->compass_angle;
    imu_data.timestamp = imu_msg->timestamp;
    new_imu = true;
}

void goal_coord_sub(const awsp_msgs::GoalCoordinates::ConstPtr& goal_msg)
{
    goal_gps_data.latitude = goal_msg->latitude;
    goal_gps_data.longitude = goal_msg->longitude;
    new_goal = true;
}

void filter_imu()
{
    std::vector<double> features;
    const uint SENSOR_NUMBER = 6;
    uint sensors[SENSOR_NUMBER] = {ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z};
    float sensor_readings[SENSOR_NUMBER];

    sensor_readings[0] = imu_data.acceleration.x;
    sensor_readings[1] = imu_data.acceleration.y;
    sensor_readings[2] = imu_data.accel_z;
    sensor_readings[3] = imu_data.gyro.x;
    sensor_readings[4] = imu_data.gyro.y;
    sensor_readings[5] = imu_data.gyro.z;

    filter_kit.set_window_size(dynr_p::low_pass_filtering_config.window_size);
    filter_kit.set_alpha_weight(dynr_p::low_pass_filtering_config.alpha_weight);

    if(dynr_p::low_pass_filtering_config.filtering_mode != dynr_p::FILTER::NONE)
    {

        filter_kit.window(sensor_readings, sensors,
                          dynr_p::low_pass_filtering_config.filtering_mode);

        features = filter_kit.get_features();

        if (dynr_p::low_pass_filtering_config.imu_gyro &&
            dynr_p::low_pass_filtering_config.imu_acc)
        {
            filtered_imu.acceleration.x = features.at(0);
            filtered_imu.acceleration.y = features.at(1);
            filtered_imu.accel_z = features.at(2);
            filtered_imu.gyro.x = features.at(3);
            filtered_imu.gyro.y = features.at(4);
            filtered_imu.gyro.z = features.at(5);
        } else if (dynr_p::low_pass_filtering_config.imu_gyro == false &&
                   dynr_p::low_pass_filtering_config.imu_acc)
        {
            filtered_imu.acceleration.x = features.at(0);
            filtered_imu.acceleration.y = features.at(1);
            filtered_imu.accel_z = features.at(2);
            filtered_imu.gyro.x = imu_data.gyro.x;
            filtered_imu.gyro.y = imu_data.gyro.y;
            filtered_imu.gyro.z = imu_data.gyro.z;

        } else if (dynr_p::low_pass_filtering_config.imu_gyro &&
                   dynr_p::low_pass_filtering_config.imu_acc == false)
        {
            filtered_imu.acceleration.x = imu_data.acceleration.x;
            filtered_imu.acceleration.y = imu_data.acceleration.y;
            filtered_imu.accel_z = imu_data.accel_z;
            filtered_imu.gyro.x = features.at(3);
            filtered_imu.gyro.y = features.at(4);
            filtered_imu.gyro.z = features.at(5);
        } else
        {
            filtered_imu = imu_data;
        }
    }
    else
    {
        filtered_imu = imu_data;
    }
}

void publish_filtered_data(awsp_msgs::SensorKitData sensor_kit_data, ros::Publisher filter_publisher)
{
    sensor_kit_data.filtered_accel_x = filtered_imu.acceleration.x;
    sensor_kit_data.filtered_accel_y = filtered_imu.acceleration.y;
    sensor_kit_data.filtered_accel_z = filtered_imu.accel_z;
    sensor_kit_data.filtered_gyro_x = filtered_imu.gyro.x;
    sensor_kit_data.filtered_gyro_y = filtered_imu.gyro.y;
    sensor_kit_data.filtered_gyro_z = filtered_imu.gyro.z;

    filter_publisher.publish(sensor_kit_data);
}

void print_pose_estimator_status(cart_pose cartesian_pose)
{
//  ROS_DEBUG_STREAM("[GPS FIX STATUS         ] " << gps_data.fix);
    ROS_DEBUG_STREAM("[GOAL GPS LAT           ] " << goal_gps_data.latitude);
    ROS_DEBUG_STREAM("[GOAL GPS LONG          ] " << goal_gps_data.longitude);

    ROS_DEBUG_STREAM("[GOAL X CART REF        ] " << cartesian_ref.position.x);
    ROS_DEBUG_STREAM("[GOAL Y CART REF        ] " << cartesian_ref.position.y);

    ROS_DEBUG_STREAM("[CURR CART POSE X       ] " << cartesian_pose.position.x);
    ROS_DEBUG_STREAM("[CURR CART POSE Y       ] " << cartesian_pose.position.y);

    ROS_DEBUG_STREAM("[IMU FILTER METHOD      ] " << dynr_p::low_pass_filtering_config.filtering_mode);
    ROS_DEBUG_STREAM("[FILTER WINDOW SIZE     ] " << dynr_p::low_pass_filtering_config.window_size);
    ROS_DEBUG_STREAM("[ALPHA WEIGHT           ] " << dynr_p::low_pass_filtering_config.alpha_weight);
    ROS_DEBUG_STREAM("[FILTER ACCELEROMETER   ] " << dynr_p::low_pass_filtering_config.imu_acc);
    ROS_DEBUG_STREAM("[FILTER GYRO            ] " << dynr_p::low_pass_filtering_config.imu_gyro);
    ROS_DEBUG_STREAM("[ESTIMATED X VEL        ] " << imu_data.vel_x);
    ROS_DEBUG_STREAM("[ESTIMATED Y VEL        ] " << imu_data.vel_y);
    ROS_DEBUG_STREAM("[ESTIMATED Z VEL        ] " << imu_data.vel_z);

    ROS_DEBUG("================================================");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_ekf_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<awsp_msgs::CartesianPose>("cartesian_pose", 1000);
    ros::Publisher filter_publisher = n.advertise<awsp_msgs::SensorKitData>("sensor_kit_data", 1000);
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Subscriber goal_sub = n.subscribe("goal_coord", 100, goal_coord_sub);
    ros::Rate loop_rate(10);

    dynamic_reconfigure::Server<awsp_pose_estimator::PoseParametersConfig> server_pose;
    dynamic_reconfigure::Server<awsp_pose_estimator::PoseParametersConfig>::CallbackType d;

    d = boost::bind(&dynr_p_callback, _1, _2);
    server_pose.setCallback(d);

    awsp_msgs::CartesianPose cart_pose_msg;
    awsp_msgs::SensorKitData sensor_kit_data_msg;

    bool is_first_gps = true;

    coordinates_2d vel;
    vel.x = 0.0;
    vel.y = 0.0;

    coordinates_2d acc;
    acc.x = 0.0;
    acc.y = 0.0;

    CartesianPose pose(gps_data, gps_data, vel, acc, 0);
    cart_pose cartesian_pose;

    bool new_data = false;

    while (ros::ok())
    {
        filter_imu();
        print_pose_estimator_status(cartesian_pose);
        if (new_goal)
        {
            cartesian_ref = pose.cartesian_pose(goal_gps_data);
            new_goal = false;
        }
        // if (is_first_gps && new_imu)
        if (is_first_gps && new_imu && new_gps)
        {
            pose = CartesianPose(gps_data, gps_data, vel, acc, imu_data.bearing);
            cartesian_pose = pose.get_last_cartesian();
            is_first_gps = false;
            new_gps = false;
            new_imu = false;
            new_data = true;
        }
        // else if (!is_first_gps)
        else if (new_gps && !is_first_gps)
        {
            cartesian_pose = pose.cartesian_pose(gps_data);
            new_gps = false;
            new_data = true;
        }
        else if (new_imu && !is_first_gps)
        {
            cartesian_pose = pose.cartesian_pose(filtered_imu);
            new_imu = false;
            new_data = true;
        }


        // Publish the states:
        cart_pose_msg.x = cartesian_pose.position.x;
        cart_pose_msg.y = cartesian_pose.position.y;
        cart_pose_msg.goal_x = cartesian_ref.position.x;
        cart_pose_msg.goal_y = cartesian_ref.position.y;
        publisher.publish(cart_pose_msg);

        publish_filtered_data(sensor_kit_data_msg, filter_publisher);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
