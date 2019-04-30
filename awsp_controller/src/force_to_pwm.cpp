#include "awsp_controller/force_to_pwm.h"

ForceToPWM::ForceToPWM()
{
  left_motor_force_ = 0;
  right_motor_force_ = 0;
}

ForceToPWM::~ForceToPWM() { }

//Calculates the PWM signal based on the desired force
float ForceToPWM::getLeftPWM(float force)
{
  float pwm = 1500.0;

  bool check = setLeftMotorForce_(force);
  if (!check) return -1;
  
  if (left_motor_force_ == 0)
    pwm = 1500;
  else if(left_motor_force_ > 0)
    pwm = FUNCTION_COEF_LEFT_MOTOR_FORWARD_[0]*left_motor_force_ + FUNCTION_COEF_LEFT_MOTOR_FORWARD_[1];
  else if(left_motor_force_ < 0)
    pwm = FUNCTION_COEF_LEFT_MOTOR_BACKWARD_[0]*left_motor_force_ + FUNCTION_COEF_LEFT_MOTOR_BACKWARD_[1];
  
  return pwm;
}

//Calculates the PWM signal based on the desired force
float ForceToPWM::getRightPWM(float force)
{
  float pwm = 1500.0;

  bool check = setRightMotorForce_(force);
  if (!check) return -1;
  
  if (right_motor_force_ == 0)
    pwm = 1500;
  else if(right_motor_force_ > 0)
    pwm = FUNCTION_COEF_RIGHT_MOTOR_FORWARD_[0]*right_motor_force_ + FUNCTION_COEF_RIGHT_MOTOR_FORWARD_[1];
  else if(right_motor_force_ < 0)
    pwm = FUNCTION_COEF_RIGHT_MOTOR_BACKWARD_[0]*right_motor_force_ + FUNCTION_COEF_RIGHT_MOTOR_BACKWARD_[1];
  
  return pwm;
}

bool ForceToPWM::setLeftMotorForce_(float left_force)
{
  if (left_force < MIN_MAX_LEFT_MOTOR_FORCE_[0])
  {
    left_motor_force_ = MIN_MAX_LEFT_MOTOR_FORCE_[0];
    return true;
  }
  else if(left_force > MIN_MAX_LEFT_MOTOR_FORCE_[1])
  {
    left_motor_force_ = MIN_MAX_LEFT_MOTOR_FORCE_[1];
    return true;
  }
  else
  {
    left_motor_force_ = left_force;
    return true;
  }
}

bool ForceToPWM::setRightMotorForce_(float right_force)
{
  if (right_force < MIN_MAX_RIGHT_MOTOR_FORCE_[0])
  {
    right_motor_force_ = MIN_MAX_RIGHT_MOTOR_FORCE_[0];
    return true;
  }
  else if (right_force > MIN_MAX_RIGHT_MOTOR_FORCE_[1])
  {
    right_motor_force_ = MIN_MAX_RIGHT_MOTOR_FORCE_[1];
    return true;
  }
  else
  {
    right_motor_force_ = right_force;
    return true;
  }
}
