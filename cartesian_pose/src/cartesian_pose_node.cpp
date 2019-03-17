#include <chrono>
#include <cstdlib>
#include <fstream>
#include <math.h>
#include <string>
#include <ros/console.h>
#include "ros/ros.h"
#include "cartesian_pose/cartesian_pose.h"
#include "gnss_l86_interface/gnss_l86_lib.h"
#include "gy_88_interface/gy_88_lib.h"
#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/CartesianLog.h"
#include "awsp_msgs/LogInstruction.h"

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

void instruction_callback(const awsp_msgs::LogInstruction::ConstPtr& instruction_msg)
{
    instruction = instruction_msg->instruction;
    new_instruction = true;
}

unsigned long long get_unix_millis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void increase_counter(std::string file_name, int number)
{
    std::ofstream counter_file;
    counter_file.open(file_name + ".count");
    counter_file << number << std::endl;
    counter_file.close();
}

int read_counter(std::string file_name)
{
    int counter = 0;
    std::ifstream counter_file(file_name);

    if (counter_file.good())
    {
        std::string line;
        getline(counter_file, line);
        counter = std::stoi(line);
    }

    counter_file.close();
    return counter;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<awsp_msgs::CartesianLog>("cartesian_log", 1000);
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Subscriber catamaran_sub = n.subscribe("log_instruction", 1000, instruction_callback);
    ros::Rate loop_rate(40);

    awsp_msgs::CartesianLog cartesian_log;

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
    std::ofstream file;
    std::string directory = "/home/ubuntu/catkin_ws/src/cartesian_pose/log/";
    std::string file_name;
    unsigned long long random = rand();
    int surge_counter = read_counter(directory + "surge_counter.count");
    int yaw_counter = read_counter(directory + "yaw_counter.count");

    while (ros::ok())
    {
        if (new_instruction)
        {
            switch(instruction)
            {
                case 2:
                    file_name = directory + "surge_damping_test_" + std::to_string(surge_counter) + ".csv";
                    surge_counter += 1;
                    increase_counter(directory + "surge_counter", surge_counter);
                    break;
                case 4:
                    file_name = directory + "yaw_damping_test_" + std::to_string(yaw_counter) + ".csv";
                    yaw_counter += 1;
                    increase_counter(directory + "yaw_counter", yaw_counter);
                    break;
            }
            new_instruction = false;
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
            cartesian_log.ready_to_log = true;
            publisher.publish(cartesian_log);
            new_gps = false;
            new_data = true;
        }
        else if (new_imu && !is_first_gps)
        {
            cartesian_pose = pose.cartesian_pose(imu_data);
            new_imu = false;
            new_data = true;
        }

        if (new_data && instruction != 0)
        {
            file.open(file_name, std::ios_base::app);
            file << pose.get_speed() << ";"
                 << pose.get_yaw_velocity() << ";"
                 << cartesian_pose.position.x << ";"
                 << cartesian_pose.position.y << ";"
                 << cartesian_pose.bearing << ";"
                 << pose.get_last_gps().latitude << ";"
                 << pose.get_last_gps().longitude << ";"
                 << imu_data.acceleration.x << ";"
                 << imu_data.acceleration.y << ";"
                 << imu_data.yaw_vel << ";"
                 << imu_data.bearing << ";"
                 << cartesian_pose.timestamp << std::endl;
            file.close();
            new_data = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
