#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <math.h>
#include "ros/ros.h"
#include <iostream>

//using Eigen::MatrixXd;

struct state_vector
{
    float x_pos;
    float y_pos;
    float vel;
    float acc;
    float heading;
    float ang_vel;
};

class KalmanFilter
{
    private:
    float t_s;                                          // time step
    float pi = 3.14159265359;                           // the number pi
    float X_u = 5;                                          // Damping in surge
    float N_r = 17;                                          // Damping in yaw
    float surge_damping_coeffs[2] = {3.16, 14.62};         // damping coefficients around the velocity switching points
    float yaw_damping_coeffs[3] = {17.16, 32.17, 37.08};   // damping coefficients around the velocity switching points
    float velocity_switch_surge = 3.6;                  // velocity point at which the damping coefficient is switched
    float velocity_switch_yaw[2] = {20, 40};               // velocity points at which the damping coefficient is switched
    float mass = 29.61;                                 // System and added mass
    float I_z = 50.22;                                  // System and added inertia around z
    Eigen::Vector2f u;                                  // Input force vector
    Eigen::VectorXf x_prior;                            // a-priori estimate (prediction of state)
    Eigen::VectorXf x_post;                             // a-posteriori estimate
    state_vector estimated_state;                       // estimated state vector
    Eigen::VectorXf z_prediction;                       // prediction of measurement
    Eigen::VectorXf z_measurement;                      // measurement

    Eigen::MatrixXf P_prior;           // System covariance
    Eigen::MatrixXf P_post;            // System covariance
    Eigen::MatrixXf Q;                 // Sensor noise covariance
    Eigen::MatrixXf R;              // Process noise covariance
    Eigen::MatrixXf H;                 // Mapping between state and measurement
    Eigen::MatrixXf Phi;                          // Dynamics
    Eigen::MatrixXf K;                                  // Kalman gain
    Eigen::MatrixXf Gamma;                         // Mapping between input and state ----- if this doesnt work, try MatrixXd Gamma(6,2);
    Eigen::MatrixXf I;
//    Eigen::MatrixXd::Identity(6, 6) I;                         // Identity matrix

    void set_damping_surge();                             // sets the damping in surge depending on the velocity
    void set_damping_yaw();                               // sets the damping in yaw depending on the angular velocity

    public:
    KalmanFilter(float time_step);
    state_vector estimate_state(float left_prop_force, float right_prop_force,
            float x_pos, float y_pos, float vel, float acc, float heading, float ang_vel);
};