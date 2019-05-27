#include "ros/ros.h"

#include "awsp_msgs/ObstacleData.h"
#include "awsp_msgs/CurrentState.h"
#include "awsp_msgs/CartesianError.h"
#include "awsp_pose_estimator/awsp_pose_estimator_lib.h"
#include "awsp_logger/awsp_logger.h"

#include "awsp_srvs/SetGoalThreshold.h"
#include "awsp_srvs/SetGNSSGoal.h"
#include "awsp_srvs/GoalToJ0.h"
#include "awsp_srvs/UseObstacleAvoidance.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>

struct ObstacleWaypoint
{
    float start_x, start_y, heading_start;
    float boat_radius = 0.5;
    float obstacle_length = boat_radius;
    float theta;
    float alpha_gain = 2, alpha;
    float w_x, w_y;
    float dist_to_waypoint;

    void set_start_coords();

} obstacle_waypoint;

struct ObstacleData
{
    float front_obstacle_dist;
    bool front_obstacle;
    bool use_obstacle = true;
} obstacle_data;

struct GoalGNSSData
{
    std::vector<std::vector<float> > waypoints;
    float goal_lat;
    float goal_long;
    bool use_waypoints = false;
    float distance_thresh;
    bool evaluate_task = false;
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


void transform_waypoint_to_j0(ros::ServiceClient goal_to_j0_client, awsp_srvs::GoalToJ0 goal_to_j0_srv)
{
    if (goal_to_j0_client.call(goal_to_j0_srv))
    {
        goal_pose.goal_x = goal_to_j0_srv.response.j0_goal_x;
        goal_pose.goal_y = goal_to_j0_srv.response.j0_goal_y;
    }
}

void transform_goal_to_j0(ros::ServiceClient goal_to_j0_client, awsp_srvs::GoalToJ0 goal_to_j0_srv)
{
    goal_to_j0_srv.request.goal_lat = goal_gnss_data.goal_lat;
    goal_to_j0_srv.request.goal_long = goal_gnss_data.goal_long;

    if (goal_to_j0_client.call(goal_to_j0_srv))
    {
        goal_pose.goal_x = goal_to_j0_srv.response.j0_goal_x;
        goal_pose.goal_y = goal_to_j0_srv.response.j0_goal_y;
    }
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
    goal_gnss_data.evaluate_task = true;

    return true;
}

bool set_goal_thresh(awsp_srvs::SetGoalThreshold::Request  &req,
         awsp_srvs::SetGoalThreshold::Response &res)
{
    res.acknowledged = true;
    goal_gnss_data.distance_thresh = (float)req.goal_thresh;
    return true;
}

bool use_obstacle_a(awsp_srvs::UseObstacleAvoidance::Request  &req,
        awsp_srvs::UseObstacleAvoidance::Response &res)
{
    res.acknowledged = true;
    obstacle_data.use_obstacle = (bool)req.use_obstacle_avoidance;
    return true;
}

void ObstacleWaypoint::set_start_coords()
{
    obstacle_waypoint.start_x = current_state.position.x;
    obstacle_waypoint.start_y = current_state.position.y;
    obstacle_waypoint.heading_start = current_state.heading;
}

void generate_obstacle_waypoint()
{
    obstacle_waypoint.theta = abs(asin(obstacle_waypoint.boat_radius / (obstacle_data.front_obstacle_dist/100)));
    obstacle_waypoint.alpha = obstacle_waypoint.theta;

    obstacle_waypoint.dist_to_waypoint = obstacle_data.front_obstacle_dist /
            cos(obstacle_waypoint.alpha * obstacle_waypoint.alpha_gain);

    obstacle_waypoint.w_x = obstacle_waypoint.start_x +
            obstacle_waypoint.dist_to_waypoint * cos(obstacle_waypoint.heading_start + obstacle_waypoint.alpha * obstacle_waypoint.alpha_gain);
    obstacle_waypoint.w_y = obstacle_waypoint.start_y +
            obstacle_waypoint.dist_to_waypoint * sin(obstacle_waypoint.heading_start + obstacle_waypoint.alpha * obstacle_waypoint.alpha_gain);
}

std::string global_log_file = "pp_log.csv";
std::string directory = "/home/ubuntu/awsp_stable_ws/src/awsp_logger/log_pp/";
Logger pp_logger(directory);

void log_pp(bool obstacle_front, bool is_there_w, bool goal_reached, float cart_error_x,
        float cart_error_y, float bearing_goal, float dist_error_obst_w, float dist_error_goal, float bearing_error)
{
    std::stringstream log_stream;
    log_stream << std::fixed << std::setprecision(7)
                << obstacle_data.front_obstacle
                << "," << obstacle_data.front_obstacle_dist
            << "," << obstacle_data.use_obstacle
            << "," << current_state.position.x
            << "," << current_state.position.y
            << "," << goal_pose.goal_x
            << "," << goal_pose.goal_y
            << "," << obstacle_waypoint.heading_start
            << "," << obstacle_waypoint.start_x
            << "," << obstacle_waypoint.start_y
            << "," << obstacle_waypoint.w_x
            << "," << obstacle_waypoint.w_y
            << "," << obstacle_waypoint.theta
            << "," << obstacle_waypoint.dist_to_waypoint
            << "," << obstacle_front
            << "," << is_there_w
            << "," << goal_reached
            << "," << cart_error_x
            << "," << cart_error_y
            << "," << bearing_goal
            << "," << bearing_error
            << "," << dist_error_obst_w
            << "," << dist_error_goal;

    pp_logger.additional_logger(log_stream.str(), global_log_file);
}

void print_pp(bool obstacle_front, bool is_there_w, bool goal_reached, float bearing_error, float dist_error_obst_w, float dist_error_goal)
{
//  ROS_INFO_STREAM("[GOAL GPS LAT           ] " << goal_gps_data.latitude);
    ROS_INFO_STREAM("[OBSTACLE FRONT         ] " << obstacle_data.front_obstacle);
    ROS_INFO_STREAM("[OBSTACLE FRONT DIST    ] " << obstacle_data.front_obstacle_dist);

    ROS_INFO_STREAM("[W X                    ] " << obstacle_waypoint.w_x);
    ROS_INFO_STREAM("[W Y                    ] " << obstacle_waypoint.w_y);
    ROS_INFO_STREAM("[BEARING ERROR          ] " << bearing_error);
    ROS_INFO_STREAM("[DIST ERROR W           ] " << dist_error_obst_w);
    ROS_INFO_STREAM("[DIST ERROR GOAL        ] " << dist_error_goal);
    ROS_INFO_STREAM("[OBSTACLE FRONT 2       ] " << obstacle_front);
    ROS_INFO_STREAM("[IS THERE W             ] " << is_there_w);
    ROS_INFO_STREAM("[GOAL REACHED           ] " << goal_reached);

    ROS_INFO("================================================");
}

void navigate_to_single_goal(awsp_msgs::CartesianError cart_error_msg,
                             ros::ServiceClient goal_to_j0_client, awsp_srvs::GoalToJ0 goal_to_j0_srv,
                             ros::Publisher cart_error_pub)
{
    bool is_transformed = false;
    bool start_coord_set = false;
    bool obstacle_front = false;
    bool is_there_w = false;
    bool goal_reached = false;
    ros::Rate loop_rate(10);
    float bearing_goal, bearing_error;
    float cart_error_x, cart_error_y, dist_error_obst_w, dist_error_goal;

    while (ros::ok())
    {
        if(obstacle_data.front_obstacle && obstacle_data.use_obstacle)
            obstacle_front = true;

        if (!is_transformed)
        {
            transform_goal_to_j0(goal_to_j0_client, goal_to_j0_srv);
            is_transformed = true;
        }

        if (!obstacle_front && !is_there_w)
        {
            cart_error_x = goal_pose.goal_x - current_state.position.x;
            cart_error_y = goal_pose.goal_y - current_state.position.y;
            dist_error_goal = sqrt(pow(cart_error_x, 2) + pow(cart_error_y, 2));
        }
        else if (obstacle_front && !is_there_w)
        {
            obstacle_waypoint.set_start_coords();
            generate_obstacle_waypoint();
            is_there_w = true;

            cart_error_x = obstacle_waypoint.w_x - current_state.position.x;
            cart_error_y = obstacle_waypoint.w_y - current_state.position.y;
            dist_error_obst_w = sqrt(pow(cart_error_x, 2) + pow(cart_error_y, 2));
            if (!obstacle_data.front_obstacle)
                obstacle_front = false;
        }
        else if (obstacle_front && is_there_w)
        {
            obstacle_waypoint.set_start_coords();
            generate_obstacle_waypoint();

            cart_error_x = obstacle_waypoint.w_x - current_state.position.x;
            cart_error_y = obstacle_waypoint.w_y - current_state.position.y;
            dist_error_obst_w = sqrt(pow(cart_error_x, 2) + pow(cart_error_y, 2));
            if (!obstacle_data.front_obstacle)
                obstacle_front = false;
        }
        else if (!obstacle_front && is_there_w)
        {
            cart_error_x = obstacle_waypoint.w_x - current_state.position.x;
            cart_error_y = obstacle_waypoint.w_y - current_state.position.y;
            dist_error_obst_w = sqrt(pow(cart_error_x, 2) + pow(cart_error_y, 2));
        }

        bearing_goal = atan2(cart_error_y, cart_error_x);
        bearing_error = bearing_goal - current_state.heading;
        cart_error_msg.bearing_error = bearing_goal - current_state.heading;
        cart_error_msg.cart_error_x = cart_error_x;
        cart_error_msg.cart_error_y = cart_error_y;

        if (dist_error_obst_w < goal_gnss_data.distance_thresh && is_there_w)
        {
            is_there_w = false;
            cart_error_msg.goal_reached = false;
        }
        else if (dist_error_goal < goal_gnss_data.distance_thresh && !is_there_w)
        {
            cart_error_msg.goal_reached = true;
            goal_reached = true;
        }
        else
            cart_error_msg.goal_reached = false;

        cart_error_pub.publish(cart_error_msg);

        if (goal_reached)
            break;

        print_pp(obstacle_front, is_there_w, goal_reached, bearing_error, dist_error_obst_w, dist_error_goal);
        log_pp(obstacle_front, is_there_w, goal_reached, cart_error_x, cart_error_y, bearing_goal,
                dist_error_obst_w, dist_error_goal, bearing_error);

        loop_rate.sleep();
        ros::spinOnce();
    }
}

void navigate_to_waypoints(awsp_msgs::CartesianError cart_error_msg,
                           ros::ServiceClient goal_to_j0_client, awsp_srvs::GoalToJ0 goal_to_j0_srv,
                           ros::Publisher cart_error_pub)
{
    std::vector<std::vector<float>> waypoints = load_gps_waypoints();

    int waypoint_counter = 0;
    bool is_transformed = false;
    bool start_coord_set = false;
    bool obstacle_front = false;
    bool goal_reached = false;
    bool is_there_w = false;
    float bearing_goal, bearing_error;
    ros::Rate loop_rate(10);
    float cart_error_x, cart_error_y, dist_error_obst_w, dist_error_goal;

    while (ros::ok())
    {
        if(obstacle_data.front_obstacle && obstacle_data.use_obstacle)
            obstacle_front = true;

        if (!is_transformed)
        {
            goal_to_j0_srv.request.goal_lat = waypoints[waypoint_counter][0];
            goal_to_j0_srv.request.goal_long = waypoints[waypoint_counter][1];
            transform_waypoint_to_j0(goal_to_j0_client, goal_to_j0_srv);
            is_transformed = true;
        }

        if (!obstacle_front && !is_there_w)
        {
            cart_error_x = goal_pose.goal_x - current_state.position.x;
            cart_error_y = goal_pose.goal_y - current_state.position.y;
            dist_error_goal = sqrt(pow(cart_error_x, 2) + pow(cart_error_y, 2));
        }
        else if (obstacle_front && !is_there_w)
        {
            obstacle_waypoint.set_start_coords();
            generate_obstacle_waypoint();
            is_there_w = true;

            cart_error_x = obstacle_waypoint.w_x - current_state.position.x;
            cart_error_y = obstacle_waypoint.w_y - current_state.position.y;
            dist_error_obst_w = sqrt(pow(cart_error_x, 2) + pow(cart_error_y, 2));
            if (!obstacle_data.front_obstacle)
                obstacle_front = false;
        }
        else if (obstacle_front && is_there_w)
        {
            obstacle_waypoint.set_start_coords();
            generate_obstacle_waypoint();

            cart_error_x = obstacle_waypoint.w_x - current_state.position.x;
            cart_error_y = obstacle_waypoint.w_y - current_state.position.y;
            dist_error_obst_w = sqrt(pow(cart_error_x, 2) + pow(cart_error_y, 2));
            if (!obstacle_data.front_obstacle)
                obstacle_front = false;
        }
        else if (!obstacle_front && is_there_w)
        {
            cart_error_x = obstacle_waypoint.w_x - current_state.position.x;
            cart_error_y = obstacle_waypoint.w_y - current_state.position.y;
            dist_error_obst_w = sqrt(pow(cart_error_x, 2) + pow(cart_error_y, 2));
        }

        bearing_goal = atan2(cart_error_y, cart_error_x);
        bearing_error = bearing_goal - current_state.heading;
        cart_error_msg.bearing_error = bearing_goal - current_state.heading;
        cart_error_msg.cart_error_x = cart_error_x;
        cart_error_msg.cart_error_y = cart_error_y;

        if (dist_error_obst_w < goal_gnss_data.distance_thresh && is_there_w)
        {
            is_there_w = false;
            cart_error_msg.goal_reached = false;
        }
        else if (dist_error_goal < goal_gnss_data.distance_thresh && !is_there_w)
        {
            waypoint_counter++;
            is_transformed = false;
            if(waypoint_counter == waypoints.size())
            {
                cart_error_msg.goal_reached = true;
                goal_reached = true;
            }
            else
                cart_error_msg.goal_reached = false;
        }
        else
            cart_error_msg.goal_reached = false;

        cart_error_pub.publish(cart_error_msg);

        if (goal_reached)
            break;

        print_pp(obstacle_front, is_there_w, goal_reached, bearing_error, dist_error_obst_w, dist_error_goal);
        log_pp(obstacle_front, is_there_w, goal_reached, cart_error_x, cart_error_y, bearing_goal,
               dist_error_obst_w, dist_error_goal, bearing_error);

        loop_rate.sleep();
        ros::spinOnce();
    }
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
    ros::ServiceServer use_obstacle_a_srv = n.advertiseService("use_obstacle_a", use_obstacle_a);

    ros::ServiceClient goal_to_j0_client = n.serviceClient<awsp_srvs::GoalToJ0>("goal_to_j0");
    awsp_srvs::GoalToJ0 goal_to_j0_srv;


    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        if (goal_gnss_data.evaluate_task)
        {
            goal_gnss_data.evaluate_task = false;
            if (!goal_gnss_data.use_waypoints)
            {
                navigate_to_single_goal(cart_error_msg, goal_to_j0_client, goal_to_j0_srv, cart_error_pub);
            } else if (goal_gnss_data.use_waypoints)
            {
                navigate_to_waypoints(cart_error_msg, goal_to_j0_client, goal_to_j0_srv, cart_error_pub);
            }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    return -1;
}
