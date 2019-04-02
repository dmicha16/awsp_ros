#include "awsp_logger/awsp_logger.h"
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

Logger::Logger() 
{
    directory_ = "/home/ubuntu/awsp_ws/src/awsp_logger/log/";
    test_counter_ = read_counter_(directory_ + "test_counter.count");
    test_counter_ += 1;
    increase_counter_(directory_ + "test_counter", test_counter_);

    temp_test_dir_ = directory_ + "test_" + std::to_string(test_counter_);
    test_dir_ =  temp_test_dir_.c_str();
    test_dir_status = mkdir(test_dir_, S_IRWXU);

    gnss_file_name_ = temp_test_dir_ + "/gnss_data.csv";
    imu_file_name_ = temp_test_dir_ + "/imu_data.csv";
}

Logger::~Logger() { }

void Logger::increase_counter_(std::string file_name, int number)
{
    std::ofstream counter_file;
    counter_file.open(file_name + ".count");
    counter_file << number << std::endl;
    counter_file.close();
}

int Logger::read_counter_(std::string file_name)
{
    int counter = 0;
    std::ifstream counter_file(file_name);

    if (counter_file.good())
    {
        std::string line;
        getline(counter_file, line);
        counter = std::stoi(line);
    }
    else 
    {
        ROS_INFO("Missing counter file. Using counter 1");
        return 0;
    }

    counter_file.close();
    return counter;
}

void Logger::set_directory(std::string path)
{
    directory_ = path;
    temp_test_dir_ = directory_ + "test_" + std::to_string(test_counter_);
    test_dir_ =  temp_test_dir_.c_str();
    test_dir_status = mkdir(test_dir_, S_IRWXU);

    gnss_file_name_ = temp_test_dir_ + "/gnss_data.csv";
    imu_file_name_ = temp_test_dir_ + "/imu_data.csv";
}

void Logger::gnss_logger(position gnss_data)
{
    gnss_file.open(gnss_file_name_, std::ios_base::app);
    gnss_file << gnss_data.latitude << ","
            << gnss_data.longitude << ","
            << gnss_data.speed << ","
            << gnss_data.true_course << ","
            << gnss_data.timestamp << std::endl;
    gnss_file.close();
}

void Logger::imu_logger(imu_data imu_data)
{
    imu_file.open (imu_file_name_, std::ios_base::app);
    imu_file << imu_data.acceleration.x <<
                "," << imu_data.acceleration.y <<
                "," << imu_data.yaw_vel <<
                "," << imu_data.bearing <<
                "," << imu_data.timestamp << std::endl;
    imu_file.close();
}




    




