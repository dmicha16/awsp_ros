#ifndef FORCE_TO_PWM_H
#define FORCE_TO_PWM_H

class ForceToPWM
{ 
  private:
    const float FUNCTION_COEF_LEFT_MOTOR_FORWARD_[2] = {30, 1500}; // the three parameters a,b in ax + b of the left motor conversion to PWM in forward motion
    const float FUNCTION_COEF_LEFT_MOTOR_BACKWARD_[2] = {30, 1500}; // the three parameters a,b in ax + b of the left motor conversion to PWM in backward motion
    const float FUNCTION_COEF_RIGHT_MOTOR_FORWARD_[2] = {-30, 1500}; // the three parameters a,b in ax + b of the right motor conversion to PWM in forward motion
    const float FUNCTION_COEF_RIGHT_MOTOR_BACKWARD_[2] = {-30, 1500}; // the three parameters a,b in ax + b of the right motor conversion to PWM in backward motion
    const float MIN_MAX_LEFT_MOTOR_FORCE_[2] = {-1.64*9.80665, 1.64*9.80665};
    const float MIN_MAX_RIGHT_MOTOR_FORCE_[2] = {-1.64*9.80665, 1.64*9.80665};
    float left_motor_force_;
    float right_motor_force_;
    bool setLeftMotorForce_(float left_force);
    bool setRightMotorForce_(float right_force);
  
  public:
    ForceToPWM();
    ~ForceToPWM();    
    float getRightPWM(float force);
    float getLeftPWM(float force);
};

#endif //FORCE_TO_PWM_H
