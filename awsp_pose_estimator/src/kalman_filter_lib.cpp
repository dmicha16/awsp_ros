#include "awsp_pose_estimator/kalman_filter_lib.h"
#include <ros/console.h>

//using Eigen::MatrixXd;

KalmanFilter::KalmanFilter(float time_step)
{
    t_s = time_step;

    x_prior.resize(6);
    x_post.resize(6);
    z_prediction.resize(6);
    z_measurement.resize(6);
    x_prior << 0, 0, 0, 0, 0, 0;
    x_post << 0, 0, 0, 0, 0, 0;
    z_prediction << 0, 0, 0, 0, 0, 0;
    z_measurement << 0, 0, 0, 0, 0, 0;

    P_prior.resize(6,6);
    P_post.resize(6,6);
    Q.resize(6,6);
    R.resize(6,6);
    H.resize(6,6);

    I.resize(6,6);
    I << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    K.resize(6,6);

    P_prior << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
    P_post << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;


    Q << 0.01, 0, 0, 0, 0, 0,
         0, 0.01, 0, 0, 0, 0,
         0, 0, 0.002, 0, 0, 0,
         0, 0, 0, 0.005, 0, 0,
         0, 0, 0, 0, 0.0001, 0,
         0, 0, 0, 0, 0, 0.0001;

    R << 0.3, 0, 0, 0, 0, 0,
         0, 0.3, 0, 0, 0, 0,
         0, 0, 4, 0, 0, 0,
         0, 0, 0, 5, 0, 0,
         0, 0, 0, 0, 10, 0,
         0, 0, 0, 0, 0, 1;

    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 3.6, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 180/pi, 0,
         0, 0, 0, 0, 0, 180/pi;

    Phi.resize(6,6);
    Gamma.resize(6,2);

    Phi << 1, 0, t_s * cos(x_prior(4)), 0.5 * (pow(t_s, 2)) * cos(x_prior(4)), 0, 0,
           0, 1, t_s * sin(x_prior(4)), 0.5 * (pow(t_s, 2)) * sin(x_prior(4)), 0, 0,
           0, 0, 1, 0.5 * t_s, 0, 0,
           0, 0, -X_u / mass, 0, 0, 0,
           0, 0, 0, 0, 1, t_s - (N_r * (pow(t_s, 2)) / (2 * I_z)),
           0, 0, 0, 0, 0, 1 - t_s * N_r / I_z;

    Gamma << 0, 0,
             0, 0,
             0, 0,
             1/mass, 1/mass,
             pow(t_s, 2)/(2 * I_z), -pow(t_s, 2)/(2 * I_z),
             t_s/I_z, -t_s/I_z;

}

void KalmanFilter::set_damping_surge()
{
    if (z_measurement(2) < velocity_switch_surge)
        X_u = surge_damping_coeffs[0];
    else
        X_u = surge_damping_coeffs[1];
}

void KalmanFilter::set_damping_yaw()
{
    if (z_measurement(5) < velocity_switch_yaw[0])
        N_r = yaw_damping_coeffs[0];
    else if (z_measurement(5) < velocity_switch_yaw[1])
        N_r= yaw_damping_coeffs[1];
    else
        N_r = yaw_damping_coeffs[2];
}

state_vector KalmanFilter::estimate_state(float left_prop_force, float right_prop_force, float x_pos, float y_pos, float vel, float acc, float heading, float ang_vel)
{
    u(0) = left_prop_force;
    u(1) = right_prop_force;
    z_measurement(0) = x_pos;
    z_measurement(1) = y_pos;
    z_measurement(2) = vel;
    z_measurement(3) = acc;
    z_measurement(4) = heading;
    z_measurement(5) = ang_vel;

    set_damping_surge();
    set_damping_yaw();

    x_prior = Phi * x_post + Gamma * u;
    z_prediction = H * x_prior;

    P_prior = Phi * P_post * Phi.transpose() + Q;

    K = P_prior * H.transpose() * (H * P_prior * H.transpose() + R).inverse();
    x_post = x_prior + K * (z_measurement - z_prediction);
    P_post = (I  - K * H) * P_prior;

    P_prior = P_post;
    x_prior = x_post;
    
    Phi << 1, 0, t_s * cos(x_prior(4)), 0.5 * (pow(t_s, 2)) * cos(x_prior(4)), 0, 0,
           0, 1, t_s * sin(x_prior(4)), 0.5 * (pow(t_s, 2)) * sin(x_prior(4)), 0, 0,
           0, 0, 1, 0.5 * t_s, 0, 0,
           0, 0, -X_u / mass, 0, 0, 0,
           0, 0, 0, 0, 1, t_s - (N_r * (pow(t_s, 2)) / (2 * I_z)),
           0, 0, 0, 0, 0, 1 - t_s * N_r / I_z;

    estimated_state.x_pos = x_post(0);
    estimated_state.y_pos = x_post(1);
    estimated_state.vel = x_post(2);
    estimated_state.acc = x_post(3);
    estimated_state.heading = x_post(4);
    estimated_state.ang_vel = x_post(5);

    return estimated_state;
}
