#include <chrono>
#include <cstdlib>
#include <fstream>
#include <math.h>
#include <string>
#include <ros/console.h>
#include "ros/ros.h"
#include "awsp_pose_estimator/awsp_pose_estimator_lib.h"
#include "awsp_pose_estimator/kalman_filter_lib.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"
#include "awsp_gy_88_interface/gy_88_lib.h"
#include "awsp_sensor_filter_kit/awsp_sensor_filter_kit_lib.h"
#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/SensorKitData.h"
#include "awsp_msgs/CurrentState.h"
#include "awsp_msgs/MotorStatus.h"
#include "awsp_pose_estimator/pose_parameters.h"
#include "awsp_logger/awsp_logger.h"

#include "awsp_srvs/GoalToJ0.h"
#include "awsp_srvs/GetConvergence.h"

#include <dynamic_reconfigure/server.h>
#include <awsp_pose_estimator/PoseParametersConfig.h>

gps_position gps_data, goal_gps_data;
imu_data imu_data, filtered_imu;

CartesianPose pose;

state_vector estimated_state;
coordinates_2d x_y_cartesian;

bool kf_converged = false;

cart_pose cartesian_ref;
bool new_imu = false;
bool new_gps = false;
bool new_goal = false;
FilterKit filter_kit(6);

struct MotorStatus
{
    float left_motor_force;
    float right_motor_force;
} motor_status;

void dynr_p_callback(awsp_pose_estimator::PoseParametersConfig &config, uint32_t level)
{
    dynr_p::low_pass_filtering_config.filtering_mode = config.low_pass_filtering_mode;
    dynr_p::low_pass_filtering_config.imu_acc = config.low_pass_imu_acc;
    dynr_p::low_pass_filtering_config.imu_gyro = config.low_pass_imu_gyro;
    dynr_p::low_pass_filtering_config.window_size = config.window_size;
    dynr_p::low_pass_filtering_config.alpha_weight = config.alpha_weight;
}

void gnss_data_callback(const awsp_msgs::GnssData::ConstPtr& gnss_msg)
{
    gps_data.latitude = gnss_msg->latitude;
    gps_data.longitude = gnss_msg->longitude;
    gps_data.timestamp = gnss_msg->timestamp;
    gps_data.true_course = gnss_msg->true_course;
    gps_data.speed = gnss_msg->speed;
    new_gps = true;
}

void motor_status_callback(const awsp_msgs::MotorStatus::ConstPtr& motor_status_msg)
{
    motor_status.left_motor_force = motor_status_msg->left_motor_force;
    motor_status.right_motor_force = motor_status_msg->right_motor_force;
}


void imu_data_callback(const awsp_msgs::Gy88Data::ConstPtr& imu_msg)
{
    imu_data.acceleration.x = floorf(imu_msg->si_accel_x * 100) / 100; // Put 10 for one decimal, 100 for 2, 1000 for 3, etc.
    imu_data.acceleration.y = -floorf(imu_msg->si_accel_y * 100) / 100;
    imu_data.accel_z = imu_msg->si_accel_z;
    imu_data.vel_x = imu_msg->vel_x;
    imu_data.vel_y = imu_msg->vel_y;
    imu_data.vel_z = imu_msg->vel_z;

    imu_data.gyro.x = imu_msg->gyro_x;
    imu_data.gyro.y = imu_msg->gyro_y;
    imu_data.gyro.z = imu_msg->gyro_z;
    imu_data.yaw_vel = imu_msg->gyro_z;
    imu_data.bearing = imu_msg->compass_angle;
    imu_data.timestamp = imu_msg->timestamp;
    new_imu = true;
}

void filter_imu()
{
    std::vector<double> features;
    const uint SENSOR_NUMBER = 6;
    uint sensors[SENSOR_NUMBER] = {ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z};
    float sensor_readings[SENSOR_NUMBER];

    sensor_readings[0] = imu_data.acceleration.x;
    sensor_readings[1] = imu_data.acceleration.y;
    sensor_readings[2] = imu_data.accel_z;
    sensor_readings[3] = imu_data.gyro.x;
    sensor_readings[4] = imu_data.gyro.y;
    sensor_readings[5] = imu_data.gyro.z;

    filter_kit.set_window_size(dynr_p::low_pass_filtering_config.window_size);
    filter_kit.set_alpha_weight(dynr_p::low_pass_filtering_config.alpha_weight);

    if(dynr_p::low_pass_filtering_config.filtering_mode != dynr_p::FILTER::NONE)
    {

        filter_kit.window(sensor_readings, sensors,
                          dynr_p::low_pass_filtering_config.filtering_mode);

        features = filter_kit.get_features();

        if (dynr_p::low_pass_filtering_config.imu_gyro &&
            dynr_p::low_pass_filtering_config.imu_acc)
        {
            filtered_imu.acceleration.x = features.at(0);
            filtered_imu.acceleration.y = features.at(1);
            filtered_imu.accel_z = features.at(2);
            filtered_imu.gyro.x = features.at(3);
            filtered_imu.gyro.y = features.at(4);
            filtered_imu.gyro.z = features.at(5);
        } else if (dynr_p::low_pass_filtering_config.imu_gyro == false &&
                   dynr_p::low_pass_filtering_config.imu_acc)
        {
            filtered_imu.acceleration.x = features.at(0);
            filtered_imu.acceleration.y = features.at(1);
            filtered_imu.accel_z = features.at(2);
            filtered_imu.gyro.x = imu_data.gyro.x;
            filtered_imu.gyro.y = imu_data.gyro.y;
            filtered_imu.gyro.z = imu_data.gyro.z;

        } else if (dynr_p::low_pass_filtering_config.imu_gyro &&
                   dynr_p::low_pass_filtering_config.imu_acc == false)
        {
            filtered_imu.acceleration.x = imu_data.acceleration.x;
            filtered_imu.acceleration.y = imu_data.acceleration.y;
            filtered_imu.accel_z = imu_data.accel_z;
            filtered_imu.gyro.x = features.at(3);
            filtered_imu.gyro.y = features.at(4);
            filtered_imu.gyro.z = features.at(5);
        } else
        {
            filtered_imu = imu_data;
        }
    }
    else
    {
        filtered_imu = imu_data;
    }

    filtered_imu.vel_x += filtered_imu.acceleration.x * 1/10;

}

void publish_filtered_data(awsp_msgs::SensorKitData sensor_kit_data, ros::Publisher filter_publisher)
{
    sensor_kit_data.filtered_accel_x = filtered_imu.acceleration.x;
    sensor_kit_data.filtered_accel_y = filtered_imu.acceleration.y;
    sensor_kit_data.filtered_accel_z = filtered_imu.accel_z;
    sensor_kit_data.filtered_gyro_x = filtered_imu.gyro.x;
    sensor_kit_data.filtered_gyro_y = filtered_imu.gyro.y;
    sensor_kit_data.filtered_gyro_z = filtered_imu.gyro.z;

    filter_publisher.publish(sensor_kit_data);
}

void print_pose_estimator_status()
{
//  ROS_DEBUG_STREAM("[GPS FIX STATUS         ] " << gps_data.fix);
    ROS_DEBUG_STREAM("[GOAL GPS LAT           ] " << goal_gps_data.latitude);
    ROS_DEBUG_STREAM("[GOAL GPS LONG          ] " << goal_gps_data.longitude);

    ROS_DEBUG_STREAM("[GOAL X CART REF        ] " << cartesian_ref.position.x);
    ROS_DEBUG_STREAM("[GOAL Y CART REF        ] " << cartesian_ref.position.y);

    ROS_DEBUG_STREAM("[EST CART POSE X        ] " << estimated_state.x_pos);
    ROS_DEBUG_STREAM("[EST CART POSE Y        ] " << estimated_state.y_pos);
    ROS_DEBUG_STREAM("[EST VELOCITY           ] " << estimated_state.vel);
    ROS_DEBUG_STREAM("[EST ACCELERATION       ] " << estimated_state.acc);
    ROS_DEBUG_STREAM("[EST HEADING            ] " << estimated_state.heading);
    ROS_DEBUG_STREAM("[EST ANGULAR VEL        ] " << estimated_state.ang_vel);

    ROS_DEBUG_STREAM("[MOTOR STATUS LEFT      ] " << motor_status.left_motor_force);
    ROS_DEBUG_STREAM("[MOTOR STATUS RIGHT     ] " << motor_status.right_motor_force);

    ROS_DEBUG_STREAM("[IMU FILTER METHOD      ] " << dynr_p::low_pass_filtering_config.filtering_mode);
    ROS_DEBUG_STREAM("[FILTER WINDOW SIZE     ] " << dynr_p::low_pass_filtering_config.window_size);
    ROS_DEBUG_STREAM("[ALPHA WEIGHT           ] " << dynr_p::low_pass_filtering_config.alpha_weight);
    ROS_DEBUG_STREAM("[FILTER ACCELEROMETER   ] " << dynr_p::low_pass_filtering_config.imu_acc);
    ROS_DEBUG_STREAM("[FILTER GYRO            ] " << dynr_p::low_pass_filtering_config.imu_gyro);

    ROS_DEBUG("================================================");
}

bool goal_to_j0(awsp_srvs::GoalToJ0::Request  &req,
                awsp_srvs::GoalToJ0::Response &res)
{
    /* this GNSS has to be transformed into the J0 frame and returned as
     * a goal of x and y in that J0 frame
     */
    goal_gps_data.latitude = (float)req.goal_lat;
    goal_gps_data.longitude = (float)req.goal_long;
    coordinates_2d j0_goals = pose.gnss_to_cartesian(goal_gps_data);
    cartesian_ref.position.x = j0_goals.x;
    cartesian_ref.position.y = j0_goals.y;
    res.j0_goal_x = j0_goals.x;
    res.j0_goal_y = j0_goals.y;
    return true;
}

bool get_convergence(awsp_srvs::GetConvergence::Request  &req,
                     awsp_srvs::GetConvergence::Response &res)
{
    //Populate this once we have determined that we are indeed converging!
    if (kf_converged)
        res.kf_is_converged = true;
    else
        res.kf_is_converged = false;

    return true;
}

std::string global_log_file = "raw_estimate.csv";
std::string directory = "/home/ubuntu/awsp_stable_ws/src/awsp_logger/log_estimate/";
Logger estimate_logger(directory);

void log_estimator(state_vector estimated_state)
{
    std::stringstream log_stream;

    log_stream << std::fixed << std::setprecision(7)
               << gps_data.latitude
               << "," << gps_data.longitude
               << "," << gps_data.speed
               << "," << gps_data.true_course
               << "," << imu_data.acceleration.x
               << "," << imu_data.acceleration.y
               << "," << imu_data.yaw_vel
               << "," << filtered_imu.acceleration.x
               << "," << filtered_imu.acceleration.y
               << "," << filtered_imu.gyro.z
               << "," << motor_status.left_motor_force
               << "," << motor_status.right_motor_force
               << "," << estimated_state.x_pos
               << "," << estimated_state.y_pos
               << "," << estimated_state.vel
               << "," << estimated_state.acc
               << "," << estimated_state.heading
               << "," << estimated_state.ang_vel;

    estimate_logger.additional_logger(log_stream.str(), global_log_file);
}


int main(int argc, char **argv)
{
    ROS_DEBUG("===== INITIALIZING POSE ESTIMATOR NODE =====");

    ros::init(argc, argv, "cartesian_pose_ekf_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    // Setup publishers
    ros::Publisher state_publisher = n.advertise<awsp_msgs::CurrentState>("current_state", 1000);
    ros::Publisher filter_publisher = n.advertise<awsp_msgs::SensorKitData>("sensor_kit_data", 1000);

    awsp_msgs::CurrentState curr_state_msg;
    awsp_msgs::SensorKitData sensor_kit_data_msg;

    // Setup subscribers
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Subscriber motor_sub = n.subscribe("motor_status", 1000, motor_status_callback);

    ros::ServiceServer goal_to_j0_srv = n.advertiseService("goal_to_j0", goal_to_j0);
    ros::ServiceServer get_conv_srv = n.advertiseService("get_convergence", get_convergence);

    dynamic_reconfigure::Server<awsp_pose_estimator::PoseParametersConfig> server_pose;
    dynamic_reconfigure::Server<awsp_pose_estimator::PoseParametersConfig>::CallbackType d;

    d = boost::bind(&dynr_p_callback, _1, _2);
    server_pose.setCallback(d);

    int convergence_var = 0;

    if (argv[1] != "")
        convergence_var = atoi(argv[1]);
    else
        convergence_var = 100;

    bool is_first_gps = true;

    coordinates_2d vel;
    vel.x = 0.0;
    vel.y = 0.0;

    coordinates_2d acc;
    acc.x = 0.0;
    acc.y = 0.0;

    // this class instance holds the J0 frame as of right now
    while (new_gps == false)
    {
        ROS_INFO("Waiting for GPS data...");
        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }

    is_first_gps = false;
//    CartesianPose pose(gps_data);
    pose.set_first_ref(gps_data);
    float time_step = 0.1;
    KalmanFilter kalman_filter(time_step);

    bool new_data = false;
    int counter = 0;

    while (ros::ok())
    {

        ROS_INFO_ONCE("Started estimating!");
        filter_imu();
//        if (new_gps && !is_first_gps)
//        {
            log_estimator(estimated_state);
            print_pose_estimator_status();
            new_gps = false;
            // cartesian_pose = pose.cartesian_pose(gps_data);
            x_y_cartesian = pose.gnss_to_cartesian(gps_data);
            estimated_state = kalman_filter.estimate_state(motor_status.left_motor_force,
                    motor_status.right_motor_force,
                    x_y_cartesian.x, x_y_cartesian.y,
                    gps_data.speed, filtered_imu.acceleration.x,
                    gps_data.true_course, filtered_imu.gyro.z);


            // Publish the states, uncomment and fill up with all the good stuff :) :
            curr_state_msg.x = estimated_state.x_pos;
            curr_state_msg.y = estimated_state.y_pos;
            curr_state_msg.vel = estimated_state.vel;
            curr_state_msg.acceleration = estimated_state.acc;
            curr_state_msg.heading = estimated_state.heading;
            curr_state_msg.angular_vel = estimated_state.ang_vel;
            state_publisher.publish(curr_state_msg);
//        }

        publish_filtered_data(sensor_kit_data_msg, filter_publisher);

        if (counter == convergence_var)
        {
            kf_converged = true;
            counter = 0;
        }
        else
        {
            counter++;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
