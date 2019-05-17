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
#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/GoalCoordinates.h"
#include "awsp_msgs/CartesianPose.h"
#include "awsp_pose_estimator/pose_parameters.h"

#include <dynamic_reconfigure/server.h>
#include <awsp_pose_estimator/PoseParametersConfig.h>

gps_position gps_data;
gps_position goal_gps_data;
imu_data imu_data;
cart_pose cartesian_ref;
bool new_imu = false;
bool new_gps = false;
bool new_goal = false;

void dynr_p_callback(awsp_pose_estimator::PoseParametersConfig &config, uint32_t level)
{
//    dynr_p::low_pass_filtering_config.filtering_mode = config.low_pass_filtering_mode;
//    dynr_p::low_pass_filtering_config.imu_acc = config.low_pass_imu_acc;
//    dynr_p::low_pass_filtering_config.imu_gyro = config.low_pass_imu_gyro;
    dynr_p::low_pass_filtering_config.window_size = config.window_size;
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
    // imu_data.acceleration.x = imu_msg->si_accel_x;
    // imu_data.acceleration.y = -imu_msg->si_accel_y;
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_ekf_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<awsp_msgs::CartesianPose>("cartesian_pose", 1000);
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Subscriber goal_sub = n.subscribe("goal_coord", 100, goal_coord_sub);
    ros::Rate loop_rate(40);

    dynamic_reconfigure::Server<awsp_pose_estimator::PoseParametersConfig> server_pose;
    dynamic_reconfigure::Server<awsp_pose_estimator::PoseParametersConfig>::CallbackType d;

    d = boost::bind(&dynr_p_callback, _1, _2);
    server_pose.setCallback(d);

    awsp_msgs::CartesianPose cart_pose_msg;

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
            cartesian_pose = pose.cartesian_pose(imu_data);
            new_imu = false;
            new_data = true;
        }


        // Publish the states:
        cart_pose_msg.x = cartesian_pose.position.x;
        cart_pose_msg.y = cartesian_pose.position.y;
        cart_pose_msg.goal_x = cartesian_ref.position.x;
        cart_pose_msg.goal_y = cartesian_ref.position.y;
        publisher.publish(cart_pose_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
