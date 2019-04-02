#include <cstdlib>
#include <fstream>
#include <math.h>
#include <string>
#include <ros/console.h>
#include "ros/ros.h"
#include <unistd.h>
#include "awsp_pose_estimator/awsp_pose_estimator.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"
#include "awsp_gy_88_interface/gy_88_lib.h"
#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/CartesianLog.h"
#include "awsp_msgs/LogInstruction.h"


class Logger
{
    private:
    int test_counter_;
    std::string directory_;
    std::string gnss_file_name_;
    std::ofstream gnss_file;

    int test_dir_status;
    std::string temp_test_dir_;
    const char* test_dir_;
    
    void increase_counter_(std::string file_name, int number);
    int read_counter_(std::string file_name);

    public:
    Logger();
    ~Logger();
    void set_directory(std::string path);
    void gnss_logger(position gnss_data);
};