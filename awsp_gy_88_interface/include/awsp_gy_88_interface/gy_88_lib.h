
#ifndef GY_88_LIB_H
#define GY_88_LIB_H

#include <cmath>
#include <bitset>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "ros/ros.h"
#include <time.h>
#include <string>
#include <string.h>
#include <iomanip>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <chrono>
#include <math.h>

// **************************************** ChipMPU6050 *****************************************

#define MPU6050_SLAVE_ADDR          0x68
#define MPU6050_PWR_MGMNT_ADDR      0x6B
#define MPU6050_REG_DATA_START      0x3b

#define MPU6050_SAMPLE_RATE_CONF    0x19
#define MPU6050_SAMPLE_RATE_1KHZ    7
#define MPU6050_SAMPLE_RATE_750MHZ  9
#define MPU6050_SAMPLE_RATE_5000MHZ 15

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40

#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42

#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_A_SCALE_2G          16384.0
#define MPU6050_A_SCALE_4G          8192
#define MPU6050_A_SCALE_8G          4096
#define MPU6050_A_SCALE_16G         2048

#define MPU6050_ACCEL_CONFIG        0x1c
#define MPU6050_ACCEL_CONFIG_2G     0
#define MPU6050_ACCEL_CONFIG_4G     8
#define MPU6050_ACCEL_CONFIG_8G     16
#define MPU6050_ACCEL_CONFIG_16G    24

#define MPU6050_G_SCALE_250         131
#define MPU6050_G_SCALE_500         65.5
#define MPU6050_G_SCALE_1000        32.8
#define MPU6050_G_SCALE_2000        16.4

#define MPU6050_GYRO_CONFIG         0x1b
#define MPU6050_GYRO_CONFIG_250     0
#define MPU6050_GYRO_CONFIG_500     8
#define MPU6050_GYRO_CONFIG_1000    16
#define MPU6050_GYRO_CONFIG_2000    24

// **************************************** HMC5883L *****************************************

#define HMC5883L_ADDRESS            0x1e

#define HCM5883L_REG_CONFIG_A       0x00
#define HCM5883L_REG_CONFIG_B       0x01
#define HMC5883L_REG_MODE           0x02
#define HMC5883L_REG_MSB_X          0x03
#define HMC5883L_REG_LSB_X          0x04
#define HMC5883L_REG_MSB_Z          0x05
#define HMC5883L_REG_LSB_Z          0x06
#define HMC5883L_REG_MSB_Y          0x07
#define HMC5883L_REG_LSB_Y          0x08
#define HMC5883L_REG_STATUS         0x09
#define HMC5883L_REG_ID_A           0x0a
#define HMC5883L_REG_ID_B           0x0b
#define HMC5883L_REG_ID_C           0x0c

#define HMC5883L_SCALE              0.92
#define HMC5883L_Y_OFFSET           1
#define HMC5883L_X_OFFSET           1

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01

#define HMC5883L_SAMPLE_RATE_CONF   0x00
#define HMC5883L_SAMPLE_RATE_75HZ   252

// **************************************** General *********************************************

#define MPU6050_CHIP                0
#define HMC5883L_CHIP               1
#define BMP085_CHIP                 2

#define PI                          3.141592
#define STANDARD_GRAVITY           9.80665

typedef unsigned long ulong_t;
typedef unsigned long long uulong_t;

struct ChipMPU6050
{
    float accel_x;
    float accel_y;
    float accel_z;

    float vel_x;
    float vel_y;
    float vel_z;

    float si_accel_x;
    float si_accel_y;
    float si_accel_z;

    float inclination_x;
    float inclination_y;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float temp;
};

struct ChipHMC5883L
{
  float compass_x;
  float compass_y;
  float compass_z;
  float compass_angle;
};

struct magnetometer
{
  float x;
  float y;
  float z;
};

class Gy88Interface
{
  public:
    Gy88Interface();
    ~Gy88Interface();

    bool connect_to_MPU6050();
    bool connect_to_HMC5883L();
    bool connect_to_BMP085();

    ChipMPU6050 get_MPU5060_data();
    ChipHMC5883L get_HMC5883L_data();

    int set_MPU6050_accel_range(int range);
    int set_MPU6050_gyro_range(int range);

    void set_dt(float loop_rate_freq);

    bool set_MPU6050_sample_rate(int sample_rate);
    bool set_HMC5883L_sample_rate(int sample_rate);

    uulong_t get_read_timestamp();

    bool read_bus(const int select_chip);

  private:
    int MPU6050_fd_;
    int HMC5883L_fd_;
    float dt_;

    float accel_scale_range_;
    float gyro_scale_range_;

    ChipMPU6050 chip_mpu6050_;
    ChipHMC5883L chip_hmc5883l_;

    uulong_t current_millis_since_epoch_;
    void set_millis_since_epoch_();
    void set_MPU6050_accel_scale_range_(int range);
    void set_MPU6050_gyro_scale_range_(int range);
    bool set_HMC5883L_scale_range_(int range);

    void read_MPU6059_accel_();
    void read_MPU6059_gyro_();
    void read_HMC5883L_compass_();

    void calculate_si_accel_();
    void estimate_inclination_();
    void estimate_velocity_();

    float calculate_compass_angle_();
    int convert_bytes_to_short_(short msb, short lsb);
};

#endif //GY_88_LIB_H