#include <ros/console.h>
#include <signal.h>
#include "gnss_l86_interface/GnssData.h"
#include "ros/ros.h"
#include "gnss_l86_interface/gnss_l86_lib.h"


int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR("Serial Port not specified");
        return 1;
    }

    char* serial_port = argv[1];

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

    ros::init(argc, argv, "gnss_l86_interface_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<gnss_l86_interface::GnssData>("gnss_data", 1000);
    ros::Rate loop_rate(2);

    int num_lines = 0;
    gnss_l86_interface::GnssData gnss_data;
    unsigned long long last_timestamp;

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
                fix_acquired = true;
            }
            else if (fix_acquired == true && last_position.fix == 0)
            {
                ROS_INFO("Fix lost!");
                fix_acquired = false;
            }

            last_timestamp = last_position.timestamp;
            gnss_data.latitude = last_position.latitude;
            gnss_data.longitude = last_position.longitude;
            gnss_data.fix = last_position.fix;
            gnss_data.number_of_satelites = last_position.number_of_satelites;
            gnss_data.horizontal_precision = last_position.horizontal_precision;
            gnss_data.altitude = last_position.altitude;
            gnss_data.timestamp = last_position.timestamp;
            publisher.publish(gnss_data);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
