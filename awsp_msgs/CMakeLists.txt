cmake_minimum_required(VERSION 2.8.3)
project(awsp_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

# Inspired from https://github.com/ros/common_msgs/tree/jade-devel/nav_msgs

add_message_files(
        DIRECTORY msg
        FILES
        Gy88Data.msg
        GnssData.msg
        CartesianLog.msg
        CartesianPose.msg
        LogInstruction.msg
        SensorKitData.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs)
