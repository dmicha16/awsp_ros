#include "catamaran_controller/test.h"
#include <iostream>
#include <stdio.h>



PropellerTest::PropellerTest(ros::NodeHandle n, int left_pin, int right_pin)
: left_esc_(left_pin), right_esc_(right_pin)
{
  log_instruction_.instruction = 0;
  cartesian_log_sub_ = n.subscribe("cartesian_log", 1000, &PropellerTest::ready_to_log_callback_, this);
  start_log_ = n.advertise<catamaran_controller::LogInstruction>("log_instruction", 1000);
  first_menu_();
}

PropellerTest::~PropellerTest() { }


///////////////////// PRIVATE ////////////////////////

void PropellerTest::ready_to_log_callback_(const cartesian_pose::CartesianLog::ConstPtr& msg)
{
  ready_to_log_ = msg->ready_to_log;
  // ROS_INFO("Message from cartesian_log topic received!");
}

void PropellerTest::first_menu_()
{
  ros::Rate loop_rate(10);
  int choice;
  bool quit = false;
  do 
  {
   if (!ready_to_log_) 
    {
     ros::spinOnce();
     continue;
    }
    
    left_esc_.setSpeed(left_esc_.NEUTRAL);
    right_esc_.setSpeed(right_esc_.NEUTRAL);
    
    ROS_INFO(" ");
    ROS_INFO("How do you want to control the boat?");
    ROS_INFO("  Press 1 for differential control");
    ROS_INFO("  Press 2 for surge damping test");
    ROS_INFO("  Press 3 for max step input");
    ROS_INFO("  Press 4 for yaw damping test");
    ROS_INFO("  Press 5 for force to pwm test");

    std::cin >> choice;
    
    switch (choice)
    {
      case 1:
        log_instruction_.instruction = 1;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        diff_control_();
        log_instruction_.instruction = 0;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        break;
      case 2:
        log_instruction_.instruction = 2;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        dual_control_();
        log_instruction_.instruction = 0;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        break;
      case 3:
        log_instruction_.instruction = 3;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        max_control_();
        log_instruction_.instruction = 0;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        break;
      case 4:
        log_instruction_.instruction = 4;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        yaw_test_control_();
        log_instruction_.instruction = 0;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        break;
      case 5:
        log_instruction_.instruction = 5;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        pwm_diff_control_();
        log_instruction_.instruction = 0;
        start_log_.publish(log_instruction_);
        ros::spinOnce();
        break;
      default:
        ROS_INFO("READ THE INSTRUCTIONS YOU IDIOT, YOUR ANSWER DOESN'T MAKE ANY SENSE!");
        quit = true;
        break;        
    }

    if (quit)
      break;

    ros::spinOnce();
    loop_rate.sleep();
  }
  while (ros::ok()); 
}

void PropellerTest::dual_control_()
{
  ROS_INFO(" ");
  ROS_INFO("------------- SURGE DAMPING TEST CONTROL MENU ----------------");
  ROS_INFO("  0 to stop");
  ROS_INFO("  1 to increase");
  ROS_INFO("  2 to decrease");
  ROS_INFO("  Any other key to quit");
  int key;
  float driving_force = 0.0;
  float left_pwm = 0.0;
  float right_pwm = 0.0;

  do
  {
    std::cin >> key;
    ROS_INFO_STREAM("You pressed " << key);
    
    if (key == 1) driving_force += INCREMENT_FORCE_;
    else if (key == 2) driving_force -= INCREMENT_FORCE_;
    else if (key == 0) driving_force = 0;
    else 
    {
      left_esc_.setSpeed(left_esc_.NEUTRAL);
      right_esc_.setSpeed(right_esc_.NEUTRAL);
      break;
    }
    
    left_pwm = converter_.getLeftPWM(driving_force);
    right_pwm = converter_.getRightPWM(driving_force);

    if (left_pwm < 0 || right_pwm < 0)
    {
      left_esc_.setSpeed(left_esc_.NEUTRAL);
      right_esc_.setSpeed(right_esc_.NEUTRAL);
      ROS_INFO("Your force was too high, resetting force to zero.");
      driving_force = 0;
    }
    else
    {
      left_esc_.setSpeed(left_pwm);
      right_esc_.setSpeed(right_pwm);
      ROS_INFO("Left PWM: %f, Right PWM: %f", left_pwm, right_pwm);
    }
  }
  while(true);
}

void PropellerTest::diff_control_()
{
  ROS_INFO(" ");
  ROS_INFO("--------- FORCE DIFFERENTIAL CONTROL MENU ------------");
  ROS_INFO("  0 to stop");
  ROS_INFO("  1 to increase LEFT motor");
  ROS_INFO("  2 to decrease LEFT motor");
  ROS_INFO("  3 to increase RIGHT motor");
  ROS_INFO("  4 to decrease RIGHT motor");
  ROS_INFO("  Any other key to quit");
  int key;
  float driving_force_left = 0.0;
  float driving_force_right = 0.0;
  float left_pwm = 0.0;
  float right_pwm = 0.0;

  do
  {
    std::cin >> key;
    ROS_INFO_STREAM("You pressed " << key);
    
    if (key == 1) driving_force_left += INCREMENT_FORCE_;
    else if (key == 2) driving_force_left -= INCREMENT_FORCE_;
    else if (key == 3) driving_force_right += INCREMENT_FORCE_;
    else if (key == 4) driving_force_right -= INCREMENT_FORCE_;
    else if (key == 0)
    {
      driving_force_left = 0;
      driving_force_right = 0;
    } 
    else 
    {
      left_esc_.setSpeed(left_esc_.NEUTRAL);
      right_esc_.setSpeed(right_esc_.NEUTRAL);
      break;
    }
    
    left_pwm = converter_.getLeftPWM(driving_force_left);
    right_pwm = converter_.getRightPWM(driving_force_right);

    if (left_pwm < 0 || right_pwm < 0)
    {
      left_esc_.setSpeed(left_esc_.NEUTRAL);
      right_esc_.setSpeed(right_esc_.NEUTRAL);
      ROS_INFO("Your force was too high, resetting force to zero.");
      driving_force_left = 0;
      driving_force_right = 0;
    }
    else
    {
      left_esc_.setSpeed(left_pwm);
      right_esc_.setSpeed(right_pwm);
      ROS_INFO("Left PWM: %f, Right PWM: %f", left_pwm, right_pwm);
    }
  }
  while(true);
}

void PropellerTest::max_control_()
{
  ROS_INFO(" ");
  ROS_INFO("--------- MAX INPUT CONTROL MENU ------------");
  ROS_INFO("  0 to stop");
  ROS_INFO("  1 to put LEFT motor to FORWARD MAX");
  ROS_INFO("  2 to put LEFT motor to BACKWARD MAX");
  ROS_INFO("  3 to put RIGHT motor to FORWARD MAX");
  ROS_INFO("  4 to put RIGHT motor to BACKWARD MAX");
  ROS_INFO("  5 to put BOTH to FORWARD MAX (FORCE)");
  ROS_INFO("  6 to put BOTH to BACKWARD MAX (FORCE)");
  ROS_INFO("  Any other key to quit");
  int key;
  float left_pwm = 0.0;
  float right_pwm = 0.0;
  float max_force = 14.5;

  do
  {
    std::cin >> key;
    ROS_INFO_STREAM("You pressed " << key);
    
    if (key == 1) left_pwm = 2500;
    else if (key == 2) left_pwm = 500;
    else if (key == 3) right_pwm = 2500;
    else if (key == 4) right_pwm = 500;
    else if (key == 5) 
    {
      right_pwm = converter_.getRightPWM(max_force);
      left_pwm = converter_.getLeftPWM(max_force);
    }
    else if (key == 6) 
    {
      right_pwm = converter_.getRightPWM(-max_force);
      left_pwm = converter_.getLeftPWM(-max_force);
    }
    else if (key == 0) 
    {
      right_pwm = 1500;
      left_pwm = 1500;
    }
    else 
    {
      left_esc_.setSpeed(left_esc_.NEUTRAL);
      right_esc_.setSpeed(right_esc_.NEUTRAL);
      break;
    }

      left_esc_.setSpeed(left_pwm);
      right_esc_.setSpeed(right_pwm);
      ROS_INFO("Left PWM: %f, Right PWM: %f", left_pwm, right_pwm);
  }
  while(true);
}

void PropellerTest::yaw_test_control_()
{
  ROS_INFO(" ");
  ROS_INFO("--------- YAW TEST CONTROL MENU ------------");
  ROS_INFO("  0 to stop");
  ROS_INFO("  1 to increase clockwise turn");
  ROS_INFO("  2 to increase counterclockwise turn");
  ROS_INFO("  Any other key to quit");
  int key;
  float driving_force_left = 0.0;
  float driving_force_right = 0.0;
  float left_pwm = 0.0;
  float right_pwm = 0.0;

  do
  {
    std::cin >> key;
    ROS_INFO_STREAM("You pressed " << key);
    
    if (key == 1)
    {
      driving_force_left += INCREMENT_FORCE_;
      driving_force_right -= INCREMENT_FORCE_;
    } 
    else if (key == 2) 
    {
      driving_force_left -= INCREMENT_FORCE_;
      driving_force_right += INCREMENT_FORCE_;
    }
    else if (key == 0)
    {
      driving_force_left = 0;
      driving_force_right = 0;
    } 
    else 
    {
      left_esc_.setSpeed(left_esc_.NEUTRAL);
      right_esc_.setSpeed(right_esc_.NEUTRAL);
      break;
    }
    
    left_pwm = converter_.getLeftPWM(driving_force_left);
    right_pwm = converter_.getRightPWM(driving_force_right);

    if (left_pwm < 0 || right_pwm < 0)
    {
      left_esc_.setSpeed(left_esc_.NEUTRAL);
      right_esc_.setSpeed(right_esc_.NEUTRAL);
      ROS_INFO("Your force was too high, resetting force to zero.");
      driving_force_left = 0;
      driving_force_right = 0;
    }
    else
    {
      left_esc_.setSpeed(left_pwm);
      right_esc_.setSpeed(right_pwm);
      ROS_INFO("Left PWM: %f, Right PWM: %f", left_pwm, right_pwm);
    }
  }
  while(true);
}

void PropellerTest::pwm_diff_control_()
{
  ROS_INFO(" ");
  ROS_INFO("--------- PWM DIFFERENTIAL CONTROL MENU ------------");
  ROS_INFO("  0 to stop");
  ROS_INFO("  1 to increase LEFT motor");
  ROS_INFO("  2 to decrease LEFT motor");
  ROS_INFO("  3 to increase RIGHT motor");
  ROS_INFO("  4 to decrease RIGHT motor");
  ROS_INFO("  Any other key to quit");
  int key;
  float left_pwm = left_esc_.NEUTRAL;
  float right_pwm = right_esc_.NEUTRAL;

  do
  {
    std::cin >> key;
    ROS_INFO_STREAM("You pressed " << key);
    
    if (key == 1) left_pwm += INCREMENT_PWM_;
    else if (key == 2) left_pwm -= INCREMENT_PWM_;
    else if (key == 3) right_pwm += INCREMENT_PWM_;
    else if (key == 4) right_pwm -= INCREMENT_PWM_;
    else if (key == 0) 
    {
      right_pwm = left_esc_.NEUTRAL;
      left_pwm = right_esc_.NEUTRAL;
    }
    else 
    {
      left_esc_.setSpeed(left_esc_.NEUTRAL);
      right_esc_.setSpeed(right_esc_.NEUTRAL);
      break;
    }
      if (left_pwm > 2500) left_pwm = 2500;
      else if (left_pwm < 500) left_pwm = 500;
      if (right_pwm > 2500) right_pwm = 2500;
      else if (right_pwm < 500) right_pwm = 500;

      left_esc_.setSpeed(left_pwm);
      right_esc_.setSpeed(right_pwm);
      ROS_INFO("Left PWM: %f, Right PWM: %f", left_pwm, right_pwm);
  }
  while(true);
}
 
