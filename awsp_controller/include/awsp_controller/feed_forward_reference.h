#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <vector>

class FeedForwardReference
{
  private:
    const float TURNING_FORCE = 5.0;
    const float MOTOR_DISPLACEMENT = 0.61;         //Distance between motor shaft and body x-axis       
    const float DAMPING_YAW = 3.5;
    const float DAMPING_SURGE = 2.7; 
    const float INERTIA_Z = 5.298577;
    const float MASS = 15;
    const float PI = 3.1415926;
    const int SAMPLES_PER_SECOND = 100; // set to 100 means sampling time is 0.01
    const float TIME_INTERVAL = 1/((float)SAMPLES_PER_SECOND);
    const int TOTAL_TIME = 500; // total seconds

    std::vector<float> time_vector;
    int time_points_();
    bool direction_of_turn_;          //True for positive turn (clockwise)
    float turning_torque_;
    float initial_bearing_;           // Initial bearing given

    float momental_linear_acc_turn (int iteration);
    float momental_linear_vel_turn (int iteration);
    float momental_linear_pos_turn (int iteration);
    float momental_angular_acc_turn (int iteration);
    float momental_angular_vel_turn (int iteration);
    float momental_angular_pos_turn (int iteration);

    std::vector<float> linear_acc_;
    std::vector<float> linear_vel_;
    std::vector<float> linear_pos_;
    std::vector<float> angular_acc_;
    std::vector<float> angular_vel_;
    std::vector<float> angular_pos_; 

  public:
    FeedForwardReference();
    ~FeedForwardReference();
    
    void set_time_vector();
    void set_direction_of_turn();
    void set_turning_torque();
    // struct initial_position {float start_x, float start_y}
    // struct goal_position {float goal_x, float goal_y};
    float angleToGoal(float current_x, float current_y, float current_bearing);
};
