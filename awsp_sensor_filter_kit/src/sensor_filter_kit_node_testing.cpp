 #include "sensor_filter_kit/sensor_filter_kit_lib.h"
 #include "gy_88_interface/gy_88_lib.h"
 #include "gy_88_interface/Gy88Data.h"
 #include "ros/ros.h"
 #include "iostream"

#include "fstream"
 
uulong_t get_millis_since_epoch()
{
  uulong_t millis_since_epoch =
    std::chrono::duration_cast<std::chrono::milliseconds>
       (std::chrono::system_clock::now().time_since_epoch()).count();

  return millis_since_epoch;
}

void record_data(uulong_t timestamp, std::vector<double> features)
{
  std::ofstream recording_file;
  recording_file.open ("/home/ubuntu/catkin_ws/src/sensor_filter_kit/sliding_window_10min.csv", \
       std::ios_base::app);
  recording_file <<
    timestamp << "," << features.at(0) << "," << features.at(1) <<
                 "," << features.at(2) << "," << features.at(3) <<
                 "," << features.at(4) << "," << features.at(5) << ",\n";
  recording_file.close();
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
  int recording_freq = 0;
  if(argc < 3)
  {
    ROS_ERROR("Missing params! Try: <recording_type> <recording_freq>.");
    return 0;
  }
  if(strcmp(argv[1], "sma") == 0 || strcmp(argv[1], "ema") == 0)
  {
    recording_freq = atoi(argv[2]);
    ROS_INFO_STREAM("Recording begins using: " << argv[1]);
    ROS_INFO_STREAM("Recording freq set to: " << recording_freq);
  }
  else
  {
    ROS_ERROR("No testing is selected, quitting.");
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
  ros::Rate loop_rate(recording_freq);

  std::vector<double> features;
  int counter = 0;
  
  ROS_INFO("Recording began.");

  while(ros::ok())
  {
    ros::spinOnce();
    sensor_readings[0] = imu_data.accel_x;
    sensor_readings[1] = imu_data.accel_y;
    sensor_readings[2] = imu_data.accel_z;
    sensor_readings[3] = imu_data.gyro_x;
    sensor_readings[4] = imu_data.gyro_y;
    sensor_readings[5] = imu_data.gyro_z;
  
    filter_kit.window(sensor_readings, sensors, SMA);
  
    features = filter_kit.get_features();
    
    uulong_t timestamp = get_millis_since_epoch();
    
    record_data(timestamp, features);
    loop_rate.sleep();
    counter++;
  }
}