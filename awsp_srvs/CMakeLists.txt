cmake_minimum_required(VERSION 2.8.3)
project(awsp_srvs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(
    DIRECTORY srv
    FILES
    GoalToJ0.srv
    GetConvergence.srv
    UseObstacleAvoidance.srv
    SetGNSSGoal.srv
    SetGoalThreshold.srv
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs )