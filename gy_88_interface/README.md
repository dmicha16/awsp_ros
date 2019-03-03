# A GY88 IMU - Raspberry PI interface as a ROS package

This repo is a ROS package to interface with the chipset on the GY88 IMU. The package features the (so far) MPU6050 and the HMC5883L chips, and most of their outputs.

# Features
So far what's included is:
Acceleration in x, y and z. Gyroscope in x, y, z and magnetometer in x, y, z and angle.

The package also features a custom ros msg:
```
float64 accel_x
float64 accel_y
float64 accel_z

float64 gyro_x
float64 gyro_y
float64 gyro_z

float64 compass_x
float64 compass_y
float64 compass_z
```

# Getting started

This package relies on the wiringPi library. Installation is simple:

```cli
$ git clone git://git.drogon.net/wiringPi
$ cd wiringPi
$ git pull origin
$ ./build
```

# Usage

To run this package simply:
`$ rosrun gy_88_interface gy_88_interface_node`
and the node will start publishing to the `gy88_data` topic. Afterwards to get the data all it takes is to subscribe to this topic.

# Authors
This package and interface has been developed by David Michalik for the use on the project Project Name.
# License
This project is licensed under the MIT License - see the LICENSE.md file for details.
