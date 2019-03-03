# A ROS filtering package

This repo is about a filtering package for various sensors used in the [AAU ASWP](https://github.com/EduPonz/awsp_collection) project. Currently the package supports any number of singals, with a varying window lenghts and two basic filtering methods.

# Features

As of right now (20-02-2019) the package only supports singular value signals, such as an axis of acceleration from an IMU. However, it is possible to use multitude of axis, applying a different window to each.

### Simple Moving Average (SMA)

This feature extraction is based on this methodology from Wikipedia. [Moving average - Wikipedia](https://en.wikipedia.org/wiki/Moving_average#Simple_moving_average)

### Exponential Moving Average (EMA)

Same applies to the exponential moving average. [Exponential Moving average - Wikipedia](https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average)

The exponential moving average only uses the previous average sample rather than calculating the average for the lenght of the window each time. It also uses a custom ALPHA_WEIGHT to determine how much should the function wiegh in the past results. This method is a lot faster computionally, but if the weight is not set according to the application's needs, it could undervalue or overvalue the past, resulting in noise average.

# Getting started

To run the package do:
`$ rosrun sensor_filter_kit sensor_filter_kit_node`

It is neccessary to pass the lenght of the window, sensor readings and the desired filter.

```C++
// Window length
const uint window_size = 100;

// number of sensors as a constant
const uint SENSOR_NUMBER =  3;

// The index of the sensors as an enum, here X_DDOT=1, Y_DDOT=2 etc.
uint sensors[SENSOR_NUMBER] = {X_DDOT, Y_DDOT, Z_DDOT};

// The readings array, same length as the number of sensors.
float sensor_readings[SENSOR_NUMBER];

// Class constructor, passing the number of sensors and the window size
FilterKit filter_kit(SENSOR_NUMBER, window_size);
```

# Usage

To pass the sensor readings to the filter class as such:

```C++

  // Storing the features for each window declare a double type std::vector
  std::vector<double> features;
  int counter = 0;

  while(ros::ok())
  {
    ros::spinOnce();
    
    // Populating the sensor readings array
    sensor_readings[0] = imu_data.x;
    sensor_readings[1] = imu_data.y;
    sensor_readings[2] = imu_data.z;
    
    // Passing the values to the filterkit window. SMA = Simple Moving Average
    filter_kit.window(sensor_readings, sensors, SMA);

    // The features for each window are returned as a vector double
    features = filter_kit.get_features();
    std::cout << counter << " -X_DDOT: - " << imu_data.x << " - " << features.at(0) << \
                            " -Y_DDOT: - " << imu_data.y << " - " << features.at(1) << \
                            " -Z_DDOT: - " << imu_data.z << " - " << features.at(2) << std::endl;

    loop_rate.sleep();
    counter++;

```
To use the EMA, simply pass the EMA as the 3rd parameter to the window as such:
`filter_kit.window(sensor_readings, sensors, EMA);`
The resulting features will no be calculated with the exponential moving average.
# Authors
This package and interface has been developed by David Michalik for the use on the project [AAU ASWP](https://github.com/EduPonz/awsp_collection).
# License
This project is licensed under the MIT License - see the LICENSE.md file for details.

[How to implement a moving average in C without a buffer? - Signal Processing Stack Exchange](https://dsp.stackexchange.com/questions/20333/how-to-implement-a-moving-average-in-c-without-a-buffer)

