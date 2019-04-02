#include <string>
#include <ros/console.h>
#include "ros/ros.h"
#include "awsp_logger.cpp"
#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/CartesianLog.h"
#include "awsp_msgs/LogInstruction.h"

position gnss_data;
imu_data imu_data_log;
bool new_imu = false;
bool new_gnss = false;
bool stop_log = false;
bool new_stop_log;

void gnss_data_callback(const awsp_msgs::GnssData::ConstPtr& gnss_msg)
{
    gnss_data.latitude = gnss_msg->latitude;
    gnss_data.longitude = gnss_msg->longitude;
    gnss_data.speed = gnss_msg->speed;
    gnss_data.true_course = gnss_msg->true_course;
    gnss_data.timestamp = gnss_msg->timestamp;
    new_gnss = true;
}

void imu_data_callback(const awsp_msgs::Gy88Data::ConstPtr& imu_msg)
{
    imu_data_log.acceleration.x = floorf(imu_msg->si_accel_x * 100) / 100; // Put 10 for one decimal, 100 for 2, 1000 for 3, etc.
    imu_data_log.acceleration.y = -floorf(imu_msg->si_accel_y * 100) / 100;
    // imu_data_log.acceleration.x = imu_msg->si_accel_x;
    // imu_data_log.acceleration.y = -imu_msg->si_accel_y;
    imu_data_log.yaw_vel = imu_msg->gyro_z;
    imu_data_log.bearing = imu_msg->compass_angle;
    imu_data_log.timestamp = imu_msg->timestamp;
    new_imu = true;
}

void stop_log_callback(const awsp_msgs::LogInstruction::ConstPtr& instruction_msg)
{
    stop_log = instruction_msg->instruction;
    new_stop_log = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "logger_node");
    ROS_INFO("Node initialized!");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    Logger logger;

    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);

    while (ros::ok())
    {
        if (new_gnss)
        {
            logger.gnss_logger(gnss_data);
            new_gnss = false;
        } 
    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}


