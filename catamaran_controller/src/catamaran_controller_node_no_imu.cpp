#include <iostream>
#include <ros/console.h>
#include <math.h>
#include "ros/ros.h"
#include "catamaran_controller/force_to_pwm.h"
#include "catamaran_controller/esc_lib.h"
#include "cartesian_pose/cartesian_pose.h"
#include "gnss_l86_interface/gnss_l86_lib.h"
#include "gnss_l86_interface/GnssData.h"
#include "gy_88_interface/gy_88_lib.h"
#include "awsp_msgs/Gy88Data.h"

gps_position gps_data;
imu_data imu_data;
bool new_imu = false;
bool new_gps = false;

void gnss_data_callback(const gnss_l86_interface::GnssData::ConstPtr& gnss_msg)
{
    gps_data.latitude = gnss_msg->latitude;
    gps_data.longitude = gnss_msg->longitude;
    gps_data.timestamp = gnss_msg->timestamp;
    new_gps = true;
}

void imu_data_callback(const awsp_msgs::Gy88Data::ConstPtr& imu_msg)
{
    // imu_data.acceleration.x = imu_msg->si_accel_x;
    // imu_data.acceleration.y = -imu_msg->si_accel_y;
    imu_data.acceleration.x = floorf(imu_msg->si_accel_x * 100) / 100;
    imu_data.acceleration.y = -floorf(imu_msg->si_accel_y * 100) / 100;
    imu_data.yaw_vel = -imu_msg->gyro_z;
    imu_data.bearing = imu_msg->compass_angle;
    imu_data.timestamp = imu_msg->timestamp;
    new_imu = true;
}

int main(int argc, char **argv)
{
    // ****************** Node Initialization ******************
    ros::init(argc, argv, "catamaran_controller_node");
    ros::NodeHandle n;
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Rate loop_rate(10);

    // ESCs and PWM converter
    ForceToPWM pwm_converter;
    esc_lib left_esc(17);
    esc_lib right_esc(27);

    // Pose estimation first construction
    coordinates_2d vel;
    coordinates_2d acc;
    CartesianPose pose_estimator(gps_data, gps_data, vel, acc, 0);
    cart_pose current_pose;

    // References
    gps_position gps_ref;
    cart_pose cartesian_ref;

    // Distance from propeller to x-axis
    float propeller_dist = 0.61;
    
    // Errors
    coordinates_2d cartesian_error;
    float distance_error;
    float bearing_goal;
    float bearing_error;

    // Proportional gains
    float linear_gain = 0.0269;
    float angular_gain = 0.167;

    // Damping coefs
    float damping_surge = 1.5437;
    float damping_yaw = 1.20348;

    // Forces and torques
    float force_drive;
    float torque_drive;
    float force_left;
    float force_right;

    // Speeds
    float linear_speed;
    float angular_speed;

    // Signals
    int pwm_left;
    int pwm_right;

    std::cout << std::endl;
    std::cout << "*****************************************************" << std::endl;
    std::cout << "* -------------- CATAMARAN CONTROLLER ------------- *" << std::endl;
    std::cout << "*****************************************************" << std::endl;
    std::cout << std::endl;
    std::cout << "Waiting for GPS signal... " << std::flush;

    bool is_first_gps = true;
    bool ref_set = false;

    while (ros::ok())
    {
        // Wait for GPS signal
        //if (is_first_gps && !new_gps) continue;
        if (!ref_set)
        {
            is_first_gps = false;
            ref_set = true;
            // Reconstruct estimator with GPS ref
            pose_estimator = CartesianPose(gps_data, gps_data, vel, acc, imu_data.bearing);
            current_pose = pose_estimator.get_last_cartesian();
            std::cout << "[acquired]" << std::endl;
        }

        std::cout << std::endl;
        std::cout << "------------- CONTROL MENU -------------" << std::endl;
        std::cout << "INTRODUCE GOAL LATITUDE: " << std::flush;
        std::cin >> gps_ref.latitude;
        std::cout << "INTRODUCE GOAL LONGITUDE: " << std::flush;
        std::cin >> gps_ref.longitude;

        std::cout << "Let's go there!" << std::endl;
        std::cout << std::endl;
        std::cout << "----------- Distance Report ------------" << std::endl;

        cartesian_ref = pose_estimator.cartesian_pose(gps_ref);

        while (ros::ok())
        {
            ros::spinOnce();
            if (new_gps)
            {
                current_pose = pose_estimator.cartesian_pose(gps_data);
                new_gps = false;

                // Calculate error
                cartesian_error.x = cartesian_ref.position.x - current_pose.position.x;
                cartesian_error.y = cartesian_ref.position.y - current_pose.position.y;
                distance_error = sqrt(pow(cartesian_error.x, 2) + pow(cartesian_error.y, 2));
                bearing_goal = atan2(cartesian_error.y, cartesian_error.x);
                bearing_error = bearing_goal - imu_data.bearing;
                if (bearing_error > M_PI) bearing_error -= 2 * M_PI;
                else if (bearing_error < -M_PI) bearing_error += 2 * M_PI;

                if (distance_error < 2) break;
                else std::cout << "Distance to destiny -> " << distance_error << std::endl;

                // Calculate speeds
                linear_speed = distance_error * linear_gain;
                angular_speed = bearing_error * angular_gain;

                // Calculate forces
                force_drive = linear_speed * damping_surge;
                torque_drive = angular_speed * damping_yaw;
                force_right = (propeller_dist * force_drive - torque_drive) / (2 * propeller_dist);
                force_left = force_drive - force_right;

                // Calculate signals
                pwm_left = pwm_converter.getLeftPWM(force_left);
                pwm_right = pwm_converter.getRightPWM(force_right);

                left_esc.setSpeed(pwm_left);
                right_esc.setSpeed(pwm_right);
            }

            loop_rate.sleep();
        }
        std::cout << "MISSION ACCOMPLISHED!" << std::endl;
    }

    return 0;
}
