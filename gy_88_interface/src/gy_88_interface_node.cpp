#include "gy_88_interface/gy_88_lib.h"
#include "ros/ros.h"
#include <iostream>
//#include "gy_88_interface/Gy88Data.h"
#include "awsp_msgs/Gy88Data.h"
#include <fstream>

int main(int argc, char **argv)
{
    int loop_rate_freq;
    if (argc < 2)
        ROS_ERROR("No loop-rate passed, defaulting to 100hz.");
    loop_rate_freq = 100;

    loop_rate_freq = atoi(argv[1]);

    ROS_INFO("Constructing IMU class..");
    Gy88Interface imu;
    ROS_INFO("Successfully constructed IMU class..");

    if(!imu.connect_to_MPU6050())
        ROS_ERROR("Couldn't connect to MPU650's I2C bus!");
    else
        ROS_INFO("Connected to MPU650's I2C bus!");

    if(!imu.connect_to_HMC5883L())
        ROS_ERROR("Couldn't connect to HMC5883L's to I2C bus!");
    else
        ROS_INFO("Connected to HMC5883L's I2C bus!");

    imu.set_MPU6050_accel_range(MPU6050_ACCEL_CONFIG_16G);
    imu.set_MPU6050_gyro_range(MPU6050_GYRO_CONFIG_2000);

    if(!imu.set_HMC5883L_sample_rate(HMC5883L_SAMPLE_RATE_75HZ))
        ROS_ERROR("Could not set compass sampling rate, running on default.");

    ros::init(argc, argv, "gy_88_interface_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<awsp_msgs::Gy88Data>("gy_88_data", 1000);
    ros::Rate loop_rate(loop_rate_freq);

    awsp_msgs::Gy88Data gy_88_data;

    ChipMPU6050 chip_mpu6050;
    ChipHMC5883L chip_hmc5883l;

    while(ros::ok())
    {
        ROS_INFO_STREAM_ONCE("Started advertising on topic gy_88_data..");

        imu.read_bus(MPU6050_CHIP);
        imu.read_bus(HMC5883L_CHIP);

        chip_mpu6050 = imu.get_MPU5060_data();
        chip_hmc5883l = imu.get_HMC5883L_data();

        gy_88_data.accel_x = chip_mpu6050.accel_x;
        gy_88_data.accel_y = chip_mpu6050.accel_y;
        gy_88_data.accel_z = chip_mpu6050.accel_z;

        gy_88_data.si_accel_x = chip_mpu6050.si_accel_x;
        gy_88_data.si_accel_y = chip_mpu6050.si_accel_y;
        gy_88_data.si_accel_z = chip_mpu6050.si_accel_z;

        gy_88_data.gyro_x = chip_mpu6050.gyro_x;
        gy_88_data.gyro_y = chip_mpu6050.gyro_y;
        gy_88_data.gyro_z = chip_mpu6050.gyro_z;

        gy_88_data.compass_x = chip_hmc5883l.compass_x;
        gy_88_data.compass_y = chip_hmc5883l.compass_y;
        gy_88_data.compass_z = chip_hmc5883l.compass_z;
        gy_88_data.compass_angle = chip_hmc5883l.compass_angle;

        gy_88_data.timestamp = imu.get_read_timestamp();

        publisher.publish(gy_88_data);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
