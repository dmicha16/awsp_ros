#include "gy_88_interface/gy_88_lib.h"
#include <iostream>
#include <bitset>

// ******************************** CONSTRUCTORS-DESTRUCTORS *******************************

Gy88Interface::Gy88Interface()
{
    wiringPiSetup();
}

Gy88Interface::~Gy88Interface() {}

// **************************************** PUBLIC *****************************************

bool Gy88Interface::connect_to_MPU6050()
{
    MPU6050_fd_ = wiringPiI2CSetup(MPU6050_SLAVE_ADDR);
    if (MPU6050_fd_ == -1)
        return false;

    wiringPiI2CWriteReg16(MPU6050_fd_, MPU6050_PWR_MGMNT_ADDR, 0);

    return true;
}

bool Gy88Interface::connect_to_HMC5883L()
{
    HMC5883L_fd_ = wiringPiI2CSetup(HMC5883L_ADDRESS);
    if (HMC5883L_fd_ == -1)
        return false;

    wiringPiI2CWriteReg16(HMC5883L_fd_, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS);

    return set_HMC5883L_scale_range_(16);
    // return true;
}

int Gy88Interface::set_MPU6050_accel_range(int range)
{
    wiringPiI2CWriteReg8(MPU6050_fd_, MPU6050_ACCEL_CONFIG, range);
    int set_range = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_ACCEL_CONFIG);

    set_MPU6050_accel_scale_range_(range);

    return set_range;
}

int Gy88Interface::set_MPU6050_gyro_range(int range)
{
    wiringPiI2CWriteReg8(MPU6050_fd_, MPU6050_GYRO_CONFIG, range);
    int set_range = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_GYRO_CONFIG);

    set_MPU6050_gyro_scale_range_(range);

    return set_range;
}

bool Gy88Interface::set_MPU6050_sample_rate(int sample_rate)
{

    wiringPiI2CWriteReg8(MPU6050_fd_, MPU6050_SAMPLE_RATE_CONF, sample_rate);
    int set_sample_rate = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_SAMPLE_RATE_CONF);

    if(set_sample_rate != sample_rate)
        return false;

    return true;
}

bool Gy88Interface::set_HMC5883L_sample_rate(int sample_rate)
{

    wiringPiI2CWriteReg8(HMC5883L_fd_, HMC5883L_SAMPLE_RATE_CONF, sample_rate);
    int set_sample_rate = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_SAMPLE_RATE_CONF);

    if(set_sample_rate != sample_rate)
        return false;

    return true;
}

ChipMPU6050 Gy88Interface::get_MPU5060_data()
{
    return chip_mpu6050_;
}

ChipHMC5883L Gy88Interface::get_HMC5883L_data()
{
    return chip_hmc5883l_;
}

bool Gy88Interface::read_bus(const int select_chip)
{

    set_millis_since_epoch_();

    switch (select_chip)
    {
        case MPU6050_CHIP:
            read_MPU6059_accel_();
            read_MPU6059_gyro_();
            break;

        case HMC5883L_CHIP:
            read_HMC5883L_compass_();
            break;
    }
}

uulong_t Gy88Interface::get_read_timestamp()
{
    return current_millis_since_epoch_;
}

// **************************************** PRIVATE ****************************************

void Gy88Interface::set_MPU6050_accel_scale_range_(int range)
{
    switch (range)
    {
        case MPU6050_ACCEL_CONFIG_2G:
            accel_scale_range_ = MPU6050_A_SCALE_2G;
            break;
        case MPU6050_ACCEL_CONFIG_4G:
            accel_scale_range_ = MPU6050_A_SCALE_4G;
            break;
        case MPU6050_ACCEL_CONFIG_8G:
            accel_scale_range_ = MPU6050_A_SCALE_8G;
            break;
        case MPU6050_ACCEL_CONFIG_16G:
            accel_scale_range_ = MPU6050_A_SCALE_16G;
            break;
    }
}

void Gy88Interface::set_MPU6050_gyro_scale_range_(int range)
{
    switch (range)
    {
        case MPU6050_GYRO_CONFIG_250:
            gyro_scale_range_ = MPU6050_G_SCALE_250;
            break;
        case MPU6050_GYRO_CONFIG_500:
            gyro_scale_range_ = MPU6050_G_SCALE_500;
            break;
        case MPU6050_GYRO_CONFIG_1000:
            gyro_scale_range_ = MPU6050_G_SCALE_1000;
            break;
        case MPU6050_GYRO_CONFIG_2000:
            gyro_scale_range_ = MPU6050_G_SCALE_2000;
            break;
    }
}

bool Gy88Interface::set_HMC5883L_scale_range_(int range)
{

    wiringPiI2CWriteReg8(HMC5883L_fd_, HCM5883L_REG_CONFIG_B, range);
    int set_range = wiringPiI2CReadReg8(HMC5883L_fd_, HCM5883L_REG_CONFIG_B);

    if(set_range != range)
        return false;

    return true;
}

void Gy88Interface::read_MPU6059_accel_()
{

    uulong_t start_time = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();
    short msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_XOUT_H);
    short lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_XOUT_L);

    chip_mpu6050_.accel_x = convert_bytes_to_short_(msb, lsb) / accel_scale_range_;

    uulong_t end_time = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();

    // std::cout << "Reading time: " << end_time - start_time << std::endl;


    msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_YOUT_H);
    lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_YOUT_L);

    chip_mpu6050_.accel_y = convert_bytes_to_short_(msb, lsb) / accel_scale_range_;

    msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_ZOUT_H);
    lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_ACCEL_ZOUT_L);

    chip_mpu6050_.accel_z = convert_bytes_to_short_(msb, lsb) / accel_scale_range_;

    calculate_si_accel_();
}

void Gy88Interface::read_MPU6059_gyro_()
{
    short msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_XOUT_H);
    short lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_XOUT_L);

    chip_mpu6050_.gyro_x = convert_bytes_to_short_(msb, lsb) / gyro_scale_range_;

    msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_YOUT_H);
    lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_YOUT_L);

    chip_mpu6050_.gyro_y = convert_bytes_to_short_(msb, lsb) / gyro_scale_range_;

    msb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_ZOUT_H);
    lsb = wiringPiI2CReadReg8(MPU6050_fd_, MPU6050_RA_GYRO_ZOUT_L);

    chip_mpu6050_.gyro_z = convert_bytes_to_short_(msb, lsb) / gyro_scale_range_;
}

void Gy88Interface::read_HMC5883L_compass_()
{
    short msb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_MSB_X);
    short lsb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_LSB_X);

    chip_hmc5883l_.compass_x = convert_bytes_to_short_(msb, lsb);

    msb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_MSB_Y);
    lsb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_LSB_Y);

    chip_hmc5883l_.compass_y = convert_bytes_to_short_(msb, lsb);

    msb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_MSB_Z);
    lsb = wiringPiI2CReadReg8(HMC5883L_fd_, HMC5883L_REG_LSB_Z);

    chip_hmc5883l_.compass_z = convert_bytes_to_short_(msb, lsb);

    chip_hmc5883l_.compass_angle = calculate_compass_angle_();
}

void Gy88Interface::calculate_si_accel_()
{
    chip_mpu6050_.si_accel_x = chip_mpu6050_.accel_x * STANDARD_GRAVITIY;
    chip_mpu6050_.si_accel_y = chip_mpu6050_.accel_y * STANDARD_GRAVITIY;
    chip_mpu6050_.si_accel_z = chip_mpu6050_.accel_z * STANDARD_GRAVITIY;
}

float Gy88Interface::calculate_compass_angle_()
{
    float angle;

    chip_hmc5883l_.compass_x = (chip_hmc5883l_.compass_x - HMC5883L_X_OFFSET) * HMC5883L_SCALE;
    chip_hmc5883l_.compass_y = (chip_hmc5883l_.compass_y - HMC5883L_Y_OFFSET) * HMC5883L_SCALE;

    angle = atan2(chip_hmc5883l_.compass_y, chip_hmc5883l_.compass_x) \
   * (180 / PI);

    return angle;
}

int Gy88Interface::convert_bytes_to_short_(short msb, short lsb)
{
    long t = msb * 0x100L + lsb;
    if(t >= 32768)
        t -= 65536;
    return (int)t;
}

void Gy88Interface::set_millis_since_epoch_()
{
    current_millis_since_epoch_ = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();
}
