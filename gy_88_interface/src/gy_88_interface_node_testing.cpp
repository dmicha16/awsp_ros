#include "gy_88_interface/gy_88_lib.h"
#include "ros/ros.h"
#include <iostream>
#include "awsp_msgs/Gy88Data.h"
#include <fstream>
#include <string.h>

uulong_t get_millis_since_epoch()
{
  uulong_t millis_since_epoch =
    std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();

  return millis_since_epoch;
}

void evaluate_results(std::vector<int> tests_avg_speed, int test_repetitions)
{

  float trials_mean, sum = 0.0, std_dev = 0.0;

  for(int n: tests_avg_speed)
  {
    sum += n;
    ROS_INFO_STREAM("The n-th element of vector test_avg_speeds: " << n);
  }

  ROS_INFO("----------");
  trials_mean = sum / test_repetitions;

  ROS_INFO_STREAM("Mean of tests which poll each register a 1000 times, with test_repetitions of: " << test_repetitions << " times, is: " << trials_mean);
  ROS_INFO_STREAM("In Hz that approximates to: " << 1000/(trials_mean/1000) << "Hz");
  ROS_INFO("----------");

  for(int n: tests_avg_speed)
    std_dev += pow(n - trials_mean, 2);

  std_dev = sqrt(std_dev / test_repetitions);

  ROS_INFO_STREAM("The standard deviation of " << test_repetitions << " tests, is: " << std_dev);
}

void test_polling_speed(int test_repetitions, Gy88Interface imu)
{
  uulong_t avg_speed, start_time, end_time;

  std::vector<int> tests_avg_speed;

  ChipMPU6050 chip_mpu6050;
  ChipHMC5883L chip_hmc5883l;

  for(size_t i = 0; i < test_repetitions; i++)
  {
    start_time = get_millis_since_epoch();

    for(size_t i = 0; i < 1000; i++)
    {
      imu.read_bus(MPU6050_CHIP);
      imu.read_bus(HMC5883L_CHIP);
      chip_mpu6050 = imu.get_MPU5060_data();
      chip_hmc5883l = imu.get_HMC5883L_data();
    }

    end_time = get_millis_since_epoch();

    avg_speed = (end_time - start_time);
    tests_avg_speed.push_back(avg_speed);
  }

  evaluate_results(tests_avg_speed, test_repetitions);
}

void record_data(uulong_t timestamp, ChipMPU6050 chip_mpu6050, ChipHMC5883L chip_hmc5883l, int recording_freq,
     std::string file_name)
{
  std::ofstream recording_file;
  recording_file.open (file_name, std::ios_base::app);
  recording_file << timestamp <<
             "," << chip_mpu6050.accel_x <<
             "," << chip_mpu6050.accel_y <<
             "," << chip_mpu6050.accel_z <<
             "," << chip_mpu6050.gyro_x <<
             "," << chip_mpu6050.gyro_y <<
             "," << chip_mpu6050.gyro_z << ",\n";
  recording_file.close();
}

int main(int argc, char *argv[])
{
  if (argc < 3)
  {
    ROS_ERROR("No testing is selected, quitting. Try: <test_type> <num_polls/recording_freq>.");
    return 0;
  }

  int num_pollings;
  int recording_freq;

  if(strcmp(argv[1], "poll") == 0)
  {
    num_pollings = atoi(argv[2]);
  }
  else if(strcmp(argv[1], "record") == 0)
  {
    recording_freq = atoi(argv[2]);
  }
  else
  {
    ROS_ERROR("No testing is selected, quitting.");
    return 0;
  }

  ROS_INFO("Constructing IMU class..");
  Gy88Interface imu;
  ROS_INFO("Successfully constructed IMU class..");

  if(!imu.connect_to_MPU6050())
  {
    ROS_ERROR("Couldn't connect to MPU650's I2C bus, quitting.");
    return 0;
  }
  else
    ROS_INFO("Connected to MPU650's I2C bus!");

  if(!imu.connect_to_HMC5883L())
  {
    ROS_ERROR("Couldn't connect to HMC5883L's to I2C bus, quitting.");
    return 0;
  }
  else
    ROS_INFO("Connected to HMC5883L's I2C bus!");

  imu.set_MPU6050_accel_range(MPU6050_ACCEL_CONFIG_16G);
  imu.set_MPU6050_gyro_range(MPU6050_GYRO_CONFIG_2000);

  if(!imu.set_HMC5883L_sample_rate(HMC5883L_SAMPLE_RATE_75HZ))
    ROS_ERROR("Could not set compass sampling rate.");

  ros::init(argc, argv, "gy_88_interface_node_testing");
  ros::NodeHandle n;
  ros::Rate loop_rate(recording_freq);

  if(strcmp(argv[1], "poll") == 0)
  {
    ROS_INFO("Began polling test!");
    test_polling_speed(num_pollings, imu);
    ROS_INFO("Polling test done, quitting.");
    return 0;
  }
  else if(strcmp(argv[1], "record") == 0)
  {
    ChipMPU6050 chip_mpu6050;
    ChipHMC5883L chip_hmc5883l;

    std::string file_name = "/home/ubuntu/catkin_ws/src/gy_88_interface/mpu6050_recording_" \
       + std::to_string(recording_freq) \
       + "_" \
       + std::to_string(get_millis_since_epoch()) + ".csv";

    while(ros::ok())
    {
      ROS_INFO_STREAM_ONCE("Started recording the data!");

      imu.read_bus(MPU6050_CHIP);
      imu.read_bus(HMC5883L_CHIP);

      chip_mpu6050 = imu.get_MPU5060_data();
      chip_hmc5883l = imu.get_HMC5883L_data();

      uulong_t timestamp = imu.get_read_timestamp();

      record_data(timestamp, chip_mpu6050, chip_hmc5883l, recording_freq, file_name);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}
