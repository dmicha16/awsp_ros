cmake_minimum_required(VERSION 2.8.3)
project(awsp_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
        DIRECTORY msg
        FILES
        MotorStatus.msg
        Gy88Data.msg
        GnssData.msg
        CartesianLog.msg
        CartesianPose.msg
        LogInstruction.msg
        SensorKitData.msg
        StateMachineStatus.msg
        ObstacleData.msg
        GoalCoordinates.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs)
