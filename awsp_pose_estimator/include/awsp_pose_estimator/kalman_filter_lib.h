#include <eigen3/Eigen/Dense>
#include <cmath>
#include <math.h>


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
    float X_u;                                          // Damping in surge
    float N_r;                                          // Damping in yaw
    float surge_damping_coeffs = [3.16, 14.62];         // damping coefficients around the velocity switching points
    float yaw_damping_coeffs = [17.16, 32.17, 37.08];   // damping coefficients around the velocity switching points
    float velocity_switch_surge = 3.6;                  // velocity point at which the damping coefficient is switched
    float velocity_switch_yaw = [20, 40];               // velocity points at which the damping coefficient is switched
    float mass = 29.61;                                 // System and added mass
    float I_z = 50.22;                                  // System and added inertia around z  
    Eigen::Vector2f u;                                  // Input force vector 
    Eigen::Vector6f x_prior;                            // a-priori estimate (prediction of state)
    Eigen::Vector6f x_post;                             // a-posteriori estimate
    state_vector estimated_state;                       // estimated state vector
    Eigen::Vector6f z_prediction;                       // prediction of measurement
    Eigen::Vector6f z_measurement;                      // measurement

    Eigen::DiagonalMatrix <float, 6> P_prior;           // System covariance
    Eigen::DiagonalMatrix <float, 6> P_post;            // System covariance
    Eigen::DiagonalMatrix <float, 6> Q;                 // Sensor noise covariance
    Eigen::DiagonalMatrix <float, 6> R;                 // Process noise covariance
    Eigen::DiagonalMatrix <float, 6> H;                 // Mapping between state and measurement
    Eigen::Matrix6f Phi;                                // Dynamics
    Eigen::Matrix6f K;                                  // Kalman gain
    Matrix <float, 6, 2> Gamma;                         // Mapping between input and state ----- if this doesnt work, try MatrixXd Gamma(6,2);
    MatrixXd::Identity(6, 6) I;                         // Identity matrix

    void set_damping_surge;                             // sets the damping in surge depending on the velocity
    void set_damping_yaw;                               // sets the damping in yaw depending on the angular velocity

    public:
    KalmanFilter(float time_step);
    state_vector estimate_state(float left_prop_force, float right_prop_force, float x_pos, float y_pos, float vel, float acc, float heading, float ang_vel);
};