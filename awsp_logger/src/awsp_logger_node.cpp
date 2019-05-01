#include <string>
#include <sstream>
#include <ros/console.h>
#include "ros/ros.h"
#include "awsp_logger/awsp_logger.h"
#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/CartesianLog.h"
#include "awsp_msgs/LogInstruction.h"

position gnss_data;
imu_data imu_data;

awsp_msgs::LogInstruction log_instruction;
bool new_imu = false;
bool new_gnss = false;
bool new_log_instruction = false;
bool log_instruction_gnss = true; // true for start to log, false for stop to log
bool log_instruction_imu = true; // true for start to log, false for stop to log

void gnss_data_callback(const awsp_msgs::GnssData::ConstPtr& gnss_msg)
{
    gnss_data.latitude = gnss_msg->latitude;
    gnss_data.longitude = gnss_msg->longitude;
    gnss_data.speed = gnss_msg->speed;
    gnss_data.true_course = gnss_msg->true_course;
    gnss_data.timestamp = gnss_msg->timestamp;
    gnss_data.fix = gnss_msg->fix;
    new_gnss = true;
}

void imu_data_callback(const awsp_msgs::Gy88Data::ConstPtr& imu_msg)
{
    imu_data.acceleration.x = floorf(imu_msg->si_accel_x * 100) / 100; // Put 10 for one decimal, 100 for 2, 1000 for 3, etc.
    imu_data.acceleration.y = -floorf(imu_msg->si_accel_y * 100) / 100;
    imu_data.yaw_vel = imu_msg->gyro_z;
    imu_data.bearing = imu_msg->compass_angle;
    imu_data.timestamp = imu_msg->timestamp;
    new_imu = true;
}

void log_instruction_callback(const awsp_msgs::LogInstruction::ConstPtr& instruction_msg)
{
    log_instruction.instruction = instruction_msg->instruction;
    new_log_instruction = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "logger_node");
    ROS_INFO("Node initialized!");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    std::string directory = "/home/ubuntu/awsp_ws/src/awsp_logger/log/";

    ROS_INFO("Creating a logger object...");
    Logger logger(directory);
    ROS_INFO("Logger object created!");

    ROS_INFO("Subscribing to GNSS and IMU data...");
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Subscriber log_instruction_sub = n.subscribe("log_instruction", 1000, log_instruction_callback);
    ROS_INFO("Subscribed successfully!");
    ROS_INFO("Listening for instructions. Logging everything...");

    while (ros::ok())
    {
        if(new_log_instruction)
        {
            switch(log_instruction.instruction)
            {
                case 0:         // stop logging
                    log_instruction_gnss = false;
                    log_instruction_imu = false;
                    ROS_INFO("Instuction is 0");
                    break;
                case 1:        // start logging
                    log_instruction_gnss = true;
                    log_instruction_imu = true;
                    ROS_INFO("Instuction is 1");
                    break;
                case 2:        // only gnss logging
                    log_instruction_gnss = true;
                    log_instruction_imu = false;
                    ROS_INFO("Instuction is 2");
                    break;
                case 3:        // only imu logging
                    log_instruction_gnss = false;
                    log_instruction_imu = true;
                    ROS_INFO("Instuction is 3");
                    break;
                default:
                    log_instruction_gnss = true;
                    log_instruction_imu = true;
                    ROS_INFO("Unidentified input. Logging all.");
                    break;
            }
            new_log_instruction = false;
        }

        if (new_gnss && log_instruction_gnss)
        {
            logger.gnss_logger(gnss_data);
            new_gnss = false;
            ROS_INFO("Logging GNSS");
        }
        if (new_imu && log_instruction_imu)
        {
            logger.imu_logger(imu_data);
            new_imu = false;
            ROS_INFO("Logging IMU");

        }
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}



