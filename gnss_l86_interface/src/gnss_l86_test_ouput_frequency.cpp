#include <chrono>
#include <cmath>
#include <ros/console.h>
#include <vector>
#include "ros/ros.h"
#include "gnss_l86_interface/gnss_l86_lib.h"

#define MAX_COUNT 100

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR("Serial Port not specified");
        return 1;
    }

    char* serial_port = argv[1];

    ROS_INFO("*******************************************************");
    ROS_INFO("*----------------- GPS FREQUENCY TEST ----------------*");
    ROS_INFO("*******************************************************");
    ROS_INFO_STREAM("Test count -> " << MAX_COUNT);
    ROS_INFO("Creating GPS Interface...");
    GnssInterface gnss;
    ROS_INFO("GPS Interface created!");

    ROS_INFO_STREAM("Opening serial connection on port " << serial_port << "...");
    if (!gnss.open_connection(serial_port, 9600))
    {
        ROS_ERROR("Cannot open connection!");
        return 1;
    }

    ROS_INFO_STREAM("Connection open on port " << serial_port);

    ros::init(argc, argv, "gnss_l86_test_output_frequency");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);

    int num_lines = 0;
    int count = 0;
    unsigned long last_timestamp = 0;
    float timer = 0;
    unsigned long elapsed_time = 0;
    std::vector<int> elapsed_times;


    ROS_INFO("Waiting for a fix...");
    bool fix_acquired = false;

    while (ros::ok())
    {
        num_lines = gnss.read_lines();
        position last_position = gnss.get_position();

        if (last_position.timestamp != last_timestamp)
        {
            if (fix_acquired == false && last_position.fix > 0)
            {
                ROS_INFO("Fix acquired!");
                ROS_INFO("---------- Starting Elapsed time measurements ---------");
                fix_acquired = true;
                last_timestamp = last_position.timestamp;
                continue;
            }
            else if (fix_acquired == true && last_position.fix == 0)
            {
                ROS_INFO("Fix lost!");
                fix_acquired = false;
            }

            if (fix_acquired)
            {
                elapsed_time = last_position.timestamp - last_timestamp;
                elapsed_times.push_back(elapsed_time);
                timer += elapsed_time;
                last_timestamp = last_position.timestamp;
                count++;
                ROS_INFO_STREAM(count << " - Elapsed time = " << elapsed_time);
            }

            if (count >= MAX_COUNT)
            {
                ROS_INFO("------------- End Elapsed time measurements -----------");
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    float avg = timer / MAX_COUNT;
    float diff = 0;
    
    for(size_t i = 0; i < elapsed_times.size(); i++)
        diff += pow((elapsed_times[i] - avg), 2);
    
    float std_deviation = sqrt((diff / (MAX_COUNT - 1)));

    float int_factor = 2.576 * (std_deviation / sqrt(MAX_COUNT));

    ROS_INFO_STREAM("AVERAGE ELAPSED TIME [ms] ------> " << avg);
    ROS_INFO_STREAM("STANDARD DEVIATION -------------> " << std_deviation);
    ROS_INFO_STREAM("99 percent confidence interval -> (" << avg - int_factor << "; " << avg + int_factor << ")");

    ROS_INFO("-------------------------------------------------------");

    return 0;
}