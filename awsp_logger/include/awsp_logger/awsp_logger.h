#ifndef AWSP_LOGGER_H
#define AWSP_LOGGER_H

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <time.h>
#include <chrono>
#include <ros/console.h>
#include "ros/ros.h"
#include <unistd.h>

#include "awsp_pose_estimator/awsp_pose_estimator.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"

#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/CartesianLog.h"
#include "awsp_msgs/LogInstruction.h"

typedef unsigned long long uulong_t;

class Logger
{
    private:
    int test_counter_;
    int test_dir_status;

    std::string directory_;
    std::string temp_test_dir_;
    const char* test_dir_;
    std::string gnss_file_name_;
    std::string imu_file_name_;
    uulong_t current_millis_since_epoch_;
    std::ofstream gnss_file_;
    std::ofstream imu_file_;
    
    void set_millis_since_epoch_();
    void increase_counter_(std::string file_name, int number);
    int read_counter_(std::string file_name);
    bool is_path_exist_(std::string dir);

    public:
    Logger(std::string directory);
    ~Logger();
    void set_directory(std::string path);
	void gnss_logger(position gnss_data);
	void imu_logger (imu_data imu_data);
    void additional_logger(std::string data_to_log, std::string file_name);
    void additional_logger(std::stringstream& data_to_log, std::string file_name);
};

#endif //AWSP_LOGGER_H