#include "awsp_logger/awsp_logger.h"
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

Logger::Logger(std::string directory) 
{
    if (!is_path_exist_(directory)) 
    {
        ROS_ERROR("No such directory.");
    }
    directory_ = directory;
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
        ROS_INFO("Missing counter file. Creating a counter file and using a counter 1...");
        std::ofstream counter_file;
        counter_file.open(file_name);
        counter_file << 0 << std::endl;
        counter_file.close();
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
    gnss_file_.open(gnss_file_name_, std::ios_base::app);
    gnss_file_ << gnss_data.timestamp << ","
            << gnss_data.latitude << ","
            << gnss_data.longitude << ","
            << gnss_data.speed << ","
            << gnss_data.true_course << std::endl;
    gnss_file_.close();
}

void Logger::imu_logger(imu_data imu_data)
{
    imu_file_.open (imu_file_name_, std::ios_base::app);
    imu_file_ << imu_data.timestamp <<
                "," << imu_data.acceleration.x <<
                "," << imu_data.acceleration.y <<
                "," << imu_data.yaw_vel <<
                "," << imu_data.bearing << std::endl;
    imu_file_.close();
}

void Logger::additional_logger(std::string data_to_log, std::string file_name)
{
    set_millis_since_epoch_();
    std::ofstream file;
    std::string file_dir = temp_test_dir_ + "/" + file_name;
    file.open (file_dir, std::ios_base::app);
    file << current_millis_since_epoch_ << "," << data_to_log << std::endl;
    file.close();
}

void Logger::additional_logger(std::stringstream& data_to_log, std::string file_name)
{
    set_millis_since_epoch_();
    std::ofstream file;
    std::string file_dir = temp_test_dir_ + "/" + file_name;
    file.open (file_dir, std::ios_base::app);
    file << current_millis_since_epoch_ << "," << data_to_log.str() << std::endl;
    file.close();
}

void Logger::set_millis_since_epoch_()
{
    current_millis_since_epoch_ = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();
}

bool Logger::is_path_exist_(const std::string dir)
{
  struct stat buffer;
  return (stat (dir.c_str(), &buffer) == 0);
}



    




