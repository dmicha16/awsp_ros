#include "ros/ros.h"

#include "awsp_msgs/ObstacleData.h"
#include "awsp_msgs/CurrentState.h"
#include "awsp_msgs/CartesianError.h"
#include "awsp_pose_estimator/awsp_pose_estimator.h"

#include "awsp_srvs/SetGoalThreshold.h"
#include "awsp_srvs/SetGNSSGoal.h"
#include "awsp_srvs/GoalToJ0.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>

struct ObstacleData
{
    float front_obstacle_dist;
    bool front_obstacle;
} obstacle_data;

struct GoalGNSSData
{
    std::vector<std::vector<float> > waypoints;
    float goal_lat;
    float goal_long;
    bool use_waypoints = false;
    float distance_thresh;
} goal_gnss_data;

cart_pose current_state, goal_pose;

/**
 * Callback function. Populates the obstacle_data struct.
 * @param obstacle_msg
 */
void obstacle_data_callback(const awsp_msgs::ObstacleData::ConstPtr obstacle_msg)
{
    obstacle_data.front_obstacle_dist = obstacle_msg->front_obstacle_dist;
    obstacle_data.front_obstacle = obstacle_msg->front_obstacle;
}

void current_state_callback(const awsp_msgs::CurrentState::ConstPtr curr_state_msg)
{
    current_state.position.x = curr_state_msg->x;
    current_state.position.y = curr_state_msg->y;
    current_state.heading = curr_state_msg->heading;
}

void define_obstacle()
{

}

void generate_right_waypoint()
{   

}

std::vector<std::vector<float>> load_gps_waypoints()
{
    std::string file_name = "/home/ubuntu/awsp_stable_ws/src/awsp_controller/waypoints/waypoints.csv";
    std::fstream gps_file(file_name.c_str());
    std::string line = "";
    std::vector<std::vector<float> > data;

    while (getline(gps_file, line))                   // read a whole line of the file
    {
        std::stringstream ss(line);                     // put it in a stringstream (internal stream)
        std::vector<float> row;
        std::string data_s;
        while (getline(ss, data_s, ',' ))           // read (string) items up to a comma
        {
            row.push_back(stod(data_s));            // use stod() to convert to double; put in row vector
        }
        if (row.size() > 0)
            data.push_back(row);    // add non-empty rows to matrix
    }
    return data;
}

bool set_gnss_goal(awsp_srvs::SetGNSSGoal::Request  &req,
        awsp_srvs::SetGNSSGoal::Response &res)
{
    res.acknowledged = true;

    goal_gnss_data.goal_lat = (float)req.goal_lat;
    goal_gnss_data.goal_long = (float)req.goal_long;
    goal_gnss_data.use_waypoints = (bool)req.use_waypoints;

    return true;
}

bool set_goal_thresh(awsp_srvs::SetGoalThreshold::Request  &req,
         awsp_srvs::SetGoalThreshold::Response &res)
{
    res.acknowledged = true;
    goal_gnss_data.distance_thresh = (float)req.goal_thresh;
    return true;
}

int main(int argc, char **argv)
{
    ROS_INFO("===== INITIALIZING PATH PLANNER NODE =====");

    ros::init(argc, argv, "awsp_path_planner_node");
    ros::NodeHandle n;

    ros::Subscriber curr_state_sub = n.subscribe("current_state", 1000, current_state_callback);
    ros::Subscriber obstacle_sub = n.subscribe("obstacle_data", 1000, obstacle_data_callback);

    ros::Publisher cart_error_pub = n.advertise<awsp_msgs::CartesianError>("cart_error", 1000);
    awsp_msgs::CartesianError cart_error_msg;

    ros::ServiceServer set_goal_thresh_srv = n.advertiseService("set_goal_thresh", set_goal_thresh);
    ros::ServiceServer set_gnss_goal_srv = n.advertiseService("set_gnss_goal", set_gnss_goal);

    ros::ServiceClient goal_to_j0_client = n.serviceClient<awsp_srvs::GoalToJ0>("goal_to_j0");
    awsp_srvs::GoalToJ0 goal_to_j0_srv;

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        /* 1) evalute if single GNSS or waypoints
         * if single GNSS -> get pose and transform GNSS to j0 goal
         * if waypoints -> store in an vector, get pose and transform first element to j0 and error
         * publish dist/heading error
         * evaluate obstacle
         */

        if(!goal_gnss_data.use_waypoints)
        {
            goal_to_j0_srv.request.goal_lat = goal_gnss_data.goal_lat;
            goal_to_j0_srv.request.goal_long = goal_gnss_data.goal_long;

            if(goal_to_j0_client.call(goal_to_j0_srv))
            {
                float cart_error_x, cart_error_y;
                goal_pose.goal_x = goal_to_j0_srv.response.j0_goal_x;
                goal_pose.goal_y = goal_to_j0_srv.response.j0_goal_y;
                cart_error_x = goal_pose.goal_x - current_state.position.x;
                cart_error_y = goal_pose.goal_y - current_state.position.y;
                cart_error_msg.bearing_error = atan2(cart_error_y, cart_error_x);
                cart_error_msg.cart_error_x = cart_error_x;
                cart_error_msg.cart_error_y = cart_error_y;
            }

            cart_error_pub.publish(cart_error_msg);
        }
        else if (goal_gnss_data.use_waypoints)
        {
            std::vector<std::vector<float>> waypoints = load_gps_waypoints();
        }


        loop_rate.sleep();
        ros::spinOnce();
    }
    return -1;
}