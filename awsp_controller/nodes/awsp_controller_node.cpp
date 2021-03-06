#include <iostream>
#include <ros/console.h>
#include <math.h>
#include "ros/ros.h"

#include "awsp_controller/force_to_pwm.h"
#include "awsp_controller/esc_lib.h"
#include "awsp_controller/state_machine.h"
#include "awsp_gnss_l86_interface/gnss_l86_lib.h"
#include "awsp_gy_88_interface/gy_88_lib.h"

#include "awsp_msgs/GnssData.h"
#include "awsp_msgs/Gy88Data.h"
#include "awsp_msgs/ObstacleData.h"
#include "awsp_msgs/StateMachineStatus.h"
#include "awsp_msgs/CartesianPose.h"
#include "awsp_msgs/GoalCoordinates.h"
#include "awsp_msgs/MotorStatus.h"
#include "awsp_msgs/CartesianError.h"

#include "awsp_srvs/SetGoalThreshold.h"
#include "awsp_srvs/SetGNSSGoal.h"
#include "awsp_srvs/UseObstacleAvoidance.h"
#include "awsp_srvs/GetConvergence.h"

#include <dynamic_reconfigure/server.h>
#include <awsp_controller/ParametersConfig.h>

/**
 * Callback function for dynamic reconfigure gui. Populates each struct corresponding to
 * groups in the gui. Handles the bitmap levels specified by the Parameters.cfg
 * @param config variable which holds all values from the gui
 * @param level bitmap level specified by the Parameters.cfg
 */
void callback(awsp_controller::ParametersConfig &config, uint32_t level) {

    switch (level) {
        case dynr::LEVEL::GAINS:
            dynr::control_gains.p_linear_gain = config.p_linear_gain;
            dynr::control_gains.d_linear_gain = config.d_linear_gain;
            dynr::control_gains.p_angular_gain = config.p_angular_gain;
            dynr::control_gains.d_angular_gain = config.d_angular_gain;
            dynr::control_gains.use_fault_detection = config.use_fault_detection;
            dynr::control_gains.use_obstacle_detector = config.use_obstacle_detector;
            break;
        case dynr::LEVEL::DYNAMIC_MODEL:
//            dynr::general_config.damping_surge = config.damping_surge;
//            dynr::general_config.damping_yaw = config.damping_yaw;
            dynr::general_config.propeller_distance = config.propeller_distance;
            break;
        case dynr::LEVEL::SYSTEM_MODE:
            dynr::system_mode.vessel = config.system_mode;
            break;
        case dynr::LEVEL::LLA_GOAL_POINTS:
            dynr::current_vessel_task.goal_latitude = config.goal_latitude;
            dynr::current_vessel_task.goal_longitude = config.goal_longitude;
            dynr::current_vessel_task.ready_to_move = config.ready_to_move;
            dynr::current_vessel_task.distance_error_tol = config.distance_error_tol;
            dynr::current_vessel_task.use_gps_waypoints = config.use_gps_waypoints;
            
            break;

        case dynr::LEVEL::BOAT_TESTING:
            dynr::boat_testing_config.ready_to_test = config.ready_to_test;
            dynr::boat_testing_config.left_motor_force = config.left_motor_force;
            dynr::boat_testing_config.right_motor_force = config.right_motor_force;
            dynr::boat_testing_config.max_force_right_motor = config.max_force_right_motor;
            dynr::boat_testing_config.max_force_left_motor = config.max_force_left_motor;
            dynr::boat_testing_config.forward_force = config.forward_force;
		    dynr::boat_testing_config.log_sensors_testing = config.log_sensors_testing;
            dynr::boat_testing_config.use_pwm = config.use_pwm_instead_of_force;
            dynr::boat_testing_config.left_motor_pwm = config.left_motor_pwm;
            dynr::boat_testing_config.right_motor_pwm = config.right_motor_pwm;
		    break;

        case dynr::LEVEL::STATE_BYPASS:
            dynr::state_bypass.bypass_2_3 = config.bypass_2_3;
            dynr::state_bypass.bypass_3_4 = config.bypass_3_4;
            dynr::state_bypass.bypass_4_2 = config.bypass_4_2;
            break;
    }
}

/**
 * Callback function. Populates the gps_data with the currently measured LLA and
 * gps fix. Also set a variable to true if this callback function has been triggered.
 * @param gnss_msg
 */
void gnss_data_callback(const awsp_msgs::GnssData::ConstPtr& gnss_msg)
{
    gps_data.latitude = gnss_msg->latitude;
    gps_data.longitude = gnss_msg->longitude;
    gps_data.timestamp = gnss_msg->timestamp;
    gps_data.fix = gnss_msg->fix;
    gps_data.true_course = gnss_msg->true_course;
    gps_data.speed = gnss_msg->speed;
    new_gps = true;
}

/**
 * Callback function. Populates the imu_data with the currently measured imu_data.
 * Also set a variable to true if this callback function has been triggered.
 * @param imu_msg
 */
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

/**
 * Callback function. Populates the obstacle_data struct.
 * @param obstacle_msg
 */
void obstacle_data_callback(const awsp_msgs::ObstacleData::ConstPtr obstacle_msg)
{
    obstacle_data.front_obstacle_dist = obstacle_msg->front_obstacle_dist;
    obstacle_data.front_obstacle = obstacle_msg->front_obstacle;
}

void cart_pose_callback(const awsp_msgs::CartesianPose::ConstPtr cart_pose_msg)
{
    cartesian_pose.position.x = cart_pose_msg->x;
    cartesian_pose.position.y = cart_pose_msg->y;
    cartesian_pose.goal_x = cart_pose_msg->goal_x;
    cartesian_pose.goal_y = cart_pose_msg->goal_y;
}

void cartesian_error_callback(const awsp_msgs::CartesianError::ConstPtr cart_error_msg)
{
    cartesian_error.cart_error_x = cart_error_msg->cart_error_x;
    cartesian_error.cart_error_y = cart_error_msg->cart_error_y;
    cartesian_error.bearing_error = cart_error_msg->bearing_error;
    cartesian_error.goal_reached = cart_error_msg->goal_reached;
}

int main(int argc, char **argv)
{
    // ****************** Node Initialization ******************

    ROS_DEBUG("===== INITIALIZING CONTROLLER NODE =====");

    ros::init(argc, argv, "awsp_controller_node");
    ros::NodeHandle n;

    // Setup subscribers
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy_88_data", 1000, imu_data_callback);
    ros::Subscriber cart_pose_sub = n.subscribe("cartesian_pose", 1000, cart_pose_callback);
    ros::Subscriber obstacle_sub = n.subscribe("obstacle_data", 1000, obstacle_data_callback);
    ros::Subscriber cart_error_sub = n.subscribe("cart_error", 1000, cartesian_error_callback);

    // Setup publishers
    ros::Publisher publisher = n.advertise<awsp_msgs::StateMachineStatus>("state_machine", 1000);
    ros::Publisher motor_publisher = n.advertise<awsp_msgs::MotorStatus>("motor_status", 1000);
    awsp_msgs::MotorStatus motor_status;
    awsp_msgs::StateMachineStatus state_machine;

    // Setup service clients
    ros::ServiceClient set_gnss_goal_client = n.serviceClient<awsp_srvs::SetGNSSGoal>("set_gnss_goal");
    ros::ServiceClient set_goal_thresh_client = n.serviceClient<awsp_srvs::SetGoalThreshold>("set_goal_thresh");
    ros::ServiceClient use_obstacle_a_client = n.serviceClient<awsp_srvs::UseObstacleAvoidance>("use_obstacle_a");
    ros::ServiceClient get_convergence_client = n.serviceClient<awsp_srvs::GetConvergence>("get_convergence");
    awsp_srvs::SetGNSSGoal set_gnss_goal_srv;
    awsp_srvs::SetGoalThreshold set_goal_thresh_srv;
    awsp_srvs::UseObstacleAvoidance use_obstacle_a_srv;
    awsp_srvs::GetConvergence get_convergence_srv;

    state_machine.current_state = state::SYSTEM_OFF;
    publisher.publish(state_machine);
    ros::Rate loop_rate(10);

    dynamic_reconfigure::Server<awsp_controller::ParametersConfig> server;
    dynamic_reconfigure::Server<awsp_controller::ParametersConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while (ros::ok())
    {
        switch (state::current_system_state)
        {
            case state::SYSTEM_OFF:
                state::current_system_state = state::system_off();
                state_machine.current_state = state::current_system_state;
                publisher.publish(state_machine);
                break;
            case state::POSE_ESTIMATION:
                state::current_system_state = state::pose_estimation(get_convergence_client, get_convergence_srv);
                state_machine.current_state = state::current_system_state;
                publisher.publish(state_machine);
                break;
            case state::GOAL_SETTING:
                state::current_system_state = state::goal_setting(set_gnss_goal_srv, set_gnss_goal_client);
                state_machine.current_state = state::current_system_state;

                set_goal_thresh_srv.request.goal_thresh = dynr::current_vessel_task.distance_error_tol;
                use_obstacle_a_srv.request.use_obstacle_avoidance = dynr::control_gains.use_obstacle_detector;

                use_obstacle_a_client.call(use_obstacle_a_srv);
                set_goal_thresh_client.call(set_goal_thresh_srv);
                publisher.publish(state_machine);
                break;
            case state::BOAT_CONTROLLER:
                state::current_system_state = state::boat_controller(motor_status, motor_publisher);
                state_machine.current_state = state::current_system_state;
                publisher.publish(state_machine);
                break;
            case state::BOAT_TESTING:
                state::current_system_state = state::boat_testing(motor_status, motor_publisher);
                state_machine.current_state = state::current_system_state;
                publisher.publish(state_machine);
                break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
