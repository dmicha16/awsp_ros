#include <ros/console.h>
#include <signal.h>
#include "ros/ros.h"
#include "gnss_l86_interface/gnss_l86_lib.h"


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

    ROS_INFO(" ");
    ROS_INFO("READ LINES");
    ROS_INFO("-------------------------------------------------");

    ros::init(argc, argv, "gnss_l86_interface_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    bool first = true;
    while (ros::ok())
    {
        std::vector<std::string> lines = gnss.read_raw_lines();

        if (first)
        {
            first = false;
            continue;
        }

        for (int i = 0; i < lines.size(); ++i)
            ROS_INFO_STREAM(lines[i]);

        loop_rate.sleep();
    }

    return 0;
}
