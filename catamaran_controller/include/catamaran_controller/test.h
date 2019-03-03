#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include "catamaran_controller/force_to_pwm.h"
#include "gnss_l86_interface/gnss_l86_lib.h"
#include "gnss_l86_interface/GnssData.h" 
#include "catamaran_controller/esc_lib.h"
#include "cartesian_pose/CartesianLog.h"
#include "catamaran_controller/LogInstruction.h"
#include <cstdlib>
#include <unistd.h>
#include <pigpiod_if2.h>

class PropellerTest
{
  private:
    ForceToPWM force_converter_;
    ros::Subscriber cartesian_log_sub_;
    ros::Publisher start_log_;
    ForceToPWM converter_;
    esc_lib left_esc_;
    esc_lib right_esc_;
    const float INCREMENT_FORCE_ = 2.5;
    const float INCREMENT_PWM_ = 100;
    catamaran_controller::LogInstruction log_instruction_;
    bool ready_to_log_;
    void ready_to_log_callback_(const cartesian_pose::CartesianLog::ConstPtr& msg);
    void first_menu_();
    void dual_control_();
    void diff_control_();
    void max_control_();   
    void pwm_diff_control_();
    void yaw_test_control_();

  public:
    PropellerTest(ros::NodeHandle n, int left_pin, int right_pin);
    ~PropellerTest();
};
