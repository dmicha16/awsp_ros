#include <ros/console.h>
#include <signal.h>
#include "awsp_msgs/GnssData.h"
#include "ros/ros.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"


int main(int argc, char **argv)
{
    int update_rate = 10;

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
    else if (argc < 4) ROS_INFO("Update rate not specified. Using 10 Hz");
    else update_rate = atoi(argv[3]);

    char* serial_port = argv[1];
    long baudrate = atol(argv[2]);
    GnssInterface gnss;

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

    if (!gnss.set_fr_mode(gnss.FITNESS_MODE))
    {
        ROS_ERROR("Cannot set fitness mode");
        return 1;
    }
    ROS_INFO("FR mode set to FITNESS_MODE");

    ros::init(argc, argv, "awsp_gnss_l86_interface_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<awsp_msgs::GnssData>("gnss_data", 1000);
    ros::Rate loop_rate(100);

    int num_lines = 0;
    awsp_msgs::GnssData gnss_data;
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

            if (last_position.true_course > M_PI)
                last_position.true_course = last_position.true_course - 2 * M_PI;

            last_timestamp = last_position.timestamp;
            gnss_data.latitude = last_position.latitude;
            gnss_data.longitude = last_position.longitude;
            gnss_data.fix = last_position.fix;
            gnss_data.number_of_satelites = last_position.number_of_satelites;
            gnss_data.horizontal_precision = last_position.horizontal_precision;
            gnss_data.altitude = last_position.altitude;
            gnss_data.timestamp = last_position.timestamp;
            gnss_data.speed = last_position.speed;
            gnss_data.true_course = last_position.true_course;
            publisher.publish(gnss_data);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
