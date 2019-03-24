#include <chrono>
#include <cmath>
#include <ros/console.h>
#include <vector>
#include "ros/ros.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"

unsigned long get_epoch()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR("Serial Port not specified");
        return 1;
    }
    else if (argc < 3)
    {
        ROS_ERROR("Baudrate not specified");
        return 1;
    }
    else if (argc < 4)
    {
        ROS_ERROR("Update rate not specified");
        return 1;
    }
    else if (argc < 5)
    {
        ROS_ERROR("Max count not specified");
        return 1;
    }

    char* serial_port = argv[1];
    long baudrate = atol(argv[2]);
    int update_rate = atoi(argv[3]);
    long max_count = atol(argv[4]);

    ROS_INFO("*******************************************************");
    ROS_INFO("*---------------- GPS READ SPEED TEST ----------------*");
    ROS_INFO("*******************************************************");
    ROS_INFO_STREAM("Test count -> " << max_count);
    ROS_INFO("Creating GPS Interface...");
    GnssInterface gnss;
    ROS_INFO("GPS Interface created!");

    ROS_INFO_STREAM("Opening serial connection on port " << serial_port << "...");
    long connection_baudrate = gnss.open_connection(serial_port);
    if (connection_baudrate == 0)
    {
        ROS_ERROR("Cannot open connection!");
        return 1;
    }

    ROS_INFO_STREAM("Connection open on port " << serial_port << ". Baudrate: " << connection_baudrate);

    if (connection_baudrate != baudrate)
    {
        ROS_INFO_STREAM("Setting baudrate to " << baudrate << "...");
        if (!gnss.set_baud_rate(baudrate))
        {
            ROS_ERROR("Wrong baud rate!");
            return 1;
        }
        
        ROS_INFO_STREAM("Closing serial connection on port " << serial_port << "...");
        gnss.close_connection();

        ROS_INFO_STREAM("Opening serial connection on port " << serial_port << "...");
        if (!gnss.open_connection(serial_port, baudrate))
        {
            ROS_ERROR("Cannot open connection!");
            return 1;
        }

        ROS_INFO_STREAM("Connection open on port " << serial_port << ". Baudrate: " << baudrate);
    }

    if (!gnss.set_update_rate(update_rate))
    {
        ROS_ERROR("Cannot set update rate");
        return 1;
    }

    ROS_INFO_STREAM("Update rate set to " << update_rate);

    ros::init(argc, argv, "gnss_l86_test_read_speed");
    ros::NodeHandle n;
    ros::Rate loop_rate(1000);

    int num_lines = 0;
    int count = 0;
    unsigned long time_1 = 0;
    unsigned long time_2 = 0;
    unsigned long read_time = 0;
    float timer = 0;
    std::vector<int> read_times;


    ROS_INFO("Waiting for a fix...");
    bool fix_acquired = false;

    ROS_INFO("------------ Starting read time measurements ----------");
    while (ros::ok())
    {
        time_1 = get_epoch();
        num_lines = gnss.read_lines();
        time_2 = get_epoch();

        if (num_lines > 0)
        {
            read_time = time_2 - time_1;
            read_times.push_back(read_time);
            timer += read_time;
            count++;

            ROS_INFO_STREAM(count << " - Read time = " << read_time);

            if (count >= max_count)
            {
                ROS_INFO("--------------- End read time measurements ------------");
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    float avg = timer / max_count;
    float diff = 0;
    
    for(size_t i = 0; i < read_times.size(); i++)
        diff += pow((read_times[i] - avg), 2);
    
    float std_deviation = sqrt((diff / (max_count - 1)));

    float int_factor = 2.576 * (std_deviation / sqrt(max_count));

    ROS_INFO_STREAM("AVERAGE READ TIME [us] ---------> " << avg);
    ROS_INFO_STREAM("STANDARD DEVIATION -------------> " << std_deviation);
    ROS_INFO_STREAM("99 percent confidence interval -> (" << avg - int_factor << "; " << avg + int_factor << ")");

    ROS_INFO("-------------------------------------------------------");

    return 0;
}