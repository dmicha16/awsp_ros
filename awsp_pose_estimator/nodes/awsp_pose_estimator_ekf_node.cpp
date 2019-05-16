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
#include "awsp_msgs/CartesianPose.h"

gps_position gps_data;
imu_data imu_data;
bool new_imu = false;
bool new_gps = false;
int instruction = 0;
bool new_instruction = false;

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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_ekf_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<awsp_msgs::CartesianPose>("cartesian_pose", 1000);
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Rate loop_rate(40);

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

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
