#include <math.h>
#include "awsp_controller/feed_forward_reference.h"

FeedForwardReference::FeedForwardReference() 
{
  
}
FeedForwardReference::~FeedForwardReference() {}


// float FeedForwardReference::angleToGoal(float current_x, float current_y, float current_bearing)
// {
//   float angle;
//   angle = atan2((goal_position(1) - current_y), goal_position(0) - current_x) - current_bearing;
//   if (angle < -PI)    
//     angle = angle + 2*PI;
//   else if (angle > PI)
//     angle = angle - 2*PI;
//   return angle; 
// }

// void FeedForwardReference::set_direction_of_turn()
// {
//   float angle = angleToGoal(initial_position(0), initial_position(1), initial_bearing_)
//   if (angle > 0)
//     direction_of_turn_ = true;
//   else
//     direction_of_turn_ = false;    
// }

// void FeedForwardReference::set_time_vector()
// {
//   vector<float> time(TOTAL_TIME*SAMPLES_PER_SECOND);

//   for (int i=0; i < TOTAL_TIME*SAMPLES_PER_SECOND; i++)
//   {
//     float j = float(i);
//     time[i] = j*TIME_INTERVAL;
//   }

// time_vector = time;
// }

void FeedForwardReference::set_turning_torque()
{
  if (direction_of_turn_ == true)
    turning_torque_ = TURNING_FORCE*MOTOR_DISPLACEMENT;
  else 
    turning_torque_ = -TURNING_FORCE*MOTOR_DISPLACEMENT;
}

float FeedForwardReference::momental_linear_acc_turn (int iteration)
{
  return (TURNING_FORCE - DAMPING_SURGE*linear_vel_[iteration-1])/(MASS + DAMPING_SURGE*TIME_INTERVAL);
}

float FeedForwardReference::momental_linear_vel_turn (int iteration)
{
  return (linear_vel_[iteration-1] + linear_acc_[iteration-1]*TIME_INTERVAL);
}

float FeedForwardReference::momental_linear_pos_turn (int iteration)
{
  return (linear_pos_[iteration-1] + linear_vel_[iteration-1]*(TIME_INTERVAL) + linear_acc_[iteration-1] * pow(TIME_INTERVAL, 2));
}

float FeedForwardReference::momental_angular_acc_turn (int iteration)
{
  return ((turning_torque_ - DAMPING_YAW*angular_vel_[iteration-1])/(INERTIA_Z + DAMPING_YAW*TIME_INTERVAL));
}

float FeedForwardReference::momental_angular_vel_turn (int iteration)
{
  return (angular_vel_[iteration-1] + angular_acc_[iteration-1]*TIME_INTERVAL);
}

float FeedForwardReference::momental_angular_pos_turn (int iteration)
{
 return (angular_pos_[iteration-1] + angular_vel_[iteration-1] * TIME_INTERVAL + angular_acc_[iteration-1] * pow(TIME_INTERVAL, 2));
}

