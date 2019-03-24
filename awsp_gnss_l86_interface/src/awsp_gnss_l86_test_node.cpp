#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string>
#include <ros/console.h>
#include "ros/ros.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"
#include "awsp_msgs/GnssData.h"

position gnss_data;
bool new_gnss = false;

// Populate gnss_data struct with information from the gnss_msg and set new_gps to true
void gnss_data_callback(const awsp_msgs::GnssData::ConstPtr& gnss_msg)
{
    gnss_data.latitude = gnss_msg->latitude;
    gnss_data.longitude = gnss_msg->longitude;
    gnss_data.speed = gnss_msg->speed;
    gnss_data.true_course = gnss_msg->true_course;
    gnss_data.timestamp = gnss_msg->timestamp;
    new_gnss = true;
}

// Get unix time in milliseconds
unsigned long long get_unix_millis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// Open the file <file_name> and write number into it
void increase_counter(std::string file_name, int number)
{
    std::ofstream counter_file;
    counter_file.open(file_name);
    counter_file << number << std::endl;
    counter_file.close();
}

// Read the content of the first line of <file_name> and return it as int if <file_name> exists
// else return 0
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
    if (argc < 2)
    {
        ROS_ERROR("Number of reads not specified");
        return 1;
    }

    int number_of_reads = atoi(argv[1]);
    if (number_of_reads != -1)
        ROS_INFO_STREAM("Reading " << number_of_reads << " GNSS datapoints");
    else
        ROS_INFO_STREAM("Reading GNSS datapoints until CTRL-C");
    ROS_INFO("----------------------------------");

    // Initialize node and subscribe to gnss_data topic
    ros::init(argc, argv, "gnss_l86_test_node");
    ros::NodeHandle n;
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Rate loop_rate(300);

    // Initialize the log file gnss_l86_test_X.csv
    std::ofstream file;
    std::string directory = "/home/ubuntu/catkin_ws/src/awsp_gnss_l86_interface/log/";
    int gnss_test_counter = read_counter(directory + "gnss_test.count");
    increase_counter(directory + "gnss_test.count", gnss_test_counter + 1);
    std::string file_name = directory + "gnss_l86_test_" + std::to_string(gnss_test_counter) + ".csv";
    file.open(file_name);
    file << "latitude;longitude;speed;true_course;timestamp" << std::endl;
    file.close();

    int i = 1;
    while (ros::ok())
    {
        // If there is new_gnss data append it to the log file
        if (new_gnss)
        {
            file.open(file_name, std::ios_base::app);
            file << std::setprecision(10)
                 << gnss_data.latitude << ";"
                 << gnss_data.longitude << ";"
                 << gnss_data.speed << ";"
                 << gnss_data.true_course << ";"
                 << gnss_data.timestamp << std::endl;
            file.close();
            ROS_INFO_STREAM(i << " -> " << gnss_data.latitude << "  " << gnss_data.longitude << "  " << gnss_data.speed << "  " << gnss_data.true_course << "  " << gnss_data.timestamp);
            i += 1;
            new_gnss = false;
        }

        if ((i > number_of_reads) && number_of_reads != -1) return 0;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
