#include "sensor_filter_kit/sensor_filter_kit_lib.h"
#include "gy_88_interface/gy_88_lib.h"
#include "gy_88_interface/Gy88Data.h"
#include "sensor_filter_kit/SensorKitData.h"
#include "ros/ros.h"
#include "iostream"

uulong_t get_millis_since_epoch()
{
  uulong_t millis_since_epoch =
       std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();
  
  return millis_since_epoch;
}

struct imu_data
{
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;      // Yaw angular acceleration [degrees/s^2]
  unsigned long long timestamp;      // Unix timestamp [ms]
};

imu_data imu_data;

void imu_data_callback(const gy_88_interface::Gy88Data::ConstPtr& imu_msg)
{
  imu_data.accel_x = imu_msg->accel_x;
  imu_data.accel_y = imu_msg->accel_y;
  imu_data.accel_z = imu_msg->accel_z;

  imu_data.gyro_x = imu_msg->gyro_x;
  imu_data.gyro_y = imu_msg->gyro_y;
  imu_data.gyro_z = imu_msg->gyro_z;
}

int main(int argc, char **argv)
{
  int publishing_freq= 0;
  if(argc < 2)
  {
    ROS_ERROR("Missing param! Try: <publishing_freq>.");
    return 0;
  }
  if(atoi(argv[1]) != 0)
  {
    publishing_freq = atoi(argv[1]);
    ROS_INFO_STREAM("Publishing freq. set to: " << publishing_freq);
  }
  else
  {
    ROS_ERROR("You must pass a recording freq. above 0, quitting.");
    return 0;
  }

  const uint window_size = 100;
  const uint SENSOR_NUMBER = 6;
  uint sensors[SENSOR_NUMBER] = {ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z};
  float sensor_readings[SENSOR_NUMBER];

  FilterKit filter_kit(SENSOR_NUMBER, window_size);

  ROS_INFO("Successfully constructed FilterKit class..");

  ros::init(argc, argv, "sensor_filter_kit_node");
  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("gy88_data", 1000, imu_data_callback);
  ros::Publisher publisher = n.advertise<sensor_filter_kit::SensorKitData>("sensor_kit_data", 1000);
  ros::Rate loop_rate(publishing_freq);
  
  sensor_filter_kit::SensorKitData sensor_kit_data;

  std::vector<double> features;
  std::cout << std::fixed;
  std::cout << std::setprecision(4);
  
  while(ros::ok())
  {
    ROS_INFO_STREAM_ONCE("Started advertising on topic sensor_kit_data..");
    
    sensor_readings[0] = imu_data.accel_x;
    sensor_readings[1] = imu_data.accel_y;
    sensor_readings[2] = imu_data.accel_z;
    sensor_readings[3] = imu_data.gyro_x;
    sensor_readings[4] = imu_data.gyro_y;
    sensor_readings[5] = imu_data.gyro_z;

    filter_kit.window(sensor_readings, sensors, SMA);

    features = filter_kit.get_features();
    
    sensor_kit_data.filtered_accel_x = features.at(0);
    sensor_kit_data.filtered_accel_y = features.at(1);
    sensor_kit_data.filtered_accel_z = features.at(2);
    sensor_kit_data.filtered_gyro_x = features.at(3);
    sensor_kit_data.filtered_gyro_y = features.at(4);
    sensor_kit_data.filtered_gyro_z = features.at(5);
  
    sensor_kit_data.timestamp = get_millis_since_epoch();
    
    publisher.publish(sensor_kit_data);
    ros::spinOnce();
    loop_rate.sleep();
  }
}