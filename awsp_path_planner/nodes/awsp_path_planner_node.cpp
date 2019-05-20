#include "ros/ros.h"

#include "awsp_msgs/ObstacleData.h"
#include "awsp_msgs/CartesianPose.h"
#include "awsp_msgs/GoalCoordinates.h"
#include "awsp_pose_estimator/awsp_pose_estimator.h"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>

struct ObstacleData
{
    float front_obstacle_dist;
    bool front_obstacle;
} obstacle_data;

cart_pose cartesian_pose;

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

void define_obstacle()
{

}

void generate_right_waypoint()
{

}

std::vector<std::vector<float> > load_gps_waypoints()
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

int main(int argc, char **argv)
{
    ROS_DEBUG("===== INITIALIZING CONTROLLER NODE =====");

    ros::init(argc, argv, "awsp_path_planner_node");
    ros::NodeHandle n;

    ros::Subscriber cart_pose_sub = n.subscribe("cartesian_pose", 1000, cart_pose_callback);
    ros::Subscriber obstacle_sub = n.subscribe("obstacle_data", 1000, obstacle_data_callback);

//    ros::Publisher coord_publisher = n.advertise<awsp_msgs::GoalCoordinates>("goal_coord", 1000);
//    awsp_msgs::GoalCoordinates goal_coordinates;

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    return -1;
}