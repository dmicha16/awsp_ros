#ifndef AWSP_POSE_ESTIMATOR_H
#define AWSP_POSE_ESTIMATOR_H

#define EARTH_R 6371000

struct gps_position
{
    float latitude;     // Parallel [degrees]
    float longitude;    // Meridian [degrees]
    unsigned long long timestamp;    // Unix timestamp [ms]
    int fix;
    float speed;
    float true_course;
};

struct coordinates_2d
{
    float x;
    float y;
};

struct gyro_3d
{
    float x;
    float y;
    float z;
};

struct cart_pose
{
    coordinates_2d position;    // Distance [m]
    float bearing;              // Heading [radians]
    unsigned long long timestamp;            // Unix timestamp [ms]
    float goal_x;
    float goal_y;
};

struct imu_data
{
    coordinates_2d acceleration;    // Liner acceleration [m/s^2]
    float accel_z;
    gyro_3d gyro;
    float yaw_vel;                  // Yaw angular acceleration [degrees/s^2]
    float bearing;                  // Magnetic heading [degrees]
    unsigned long long timestamp;                // Unix timestamp [ms]
};

class CartesianPose
{
    private:
        gps_position magnetic_north_;
        gps_position ref_;
        gps_position last_gps_;
        cart_pose last_cartesian_;
        coordinates_2d last_gps_cart_;
        coordinates_2d last_velocity_;
        coordinates_2d last_acceleration_;
        float declination_;
        float last_bearing_;
        float last_yaw_vel_;
        float last_yaw_acc_;
        unsigned long long last_gps_timestamp_;
        float bearing_(gps_position origin, gps_position destination);
        coordinates_2d cartesian_position_(gps_position gps);
        float magnetic_declination_(gps_position gps, gps_position magnetic_north);
        bool check_angle_(float degrees);
        bool check_gps_position_(gps_position gps);
        float degrees_(float radians);
        gps_position degrees_(gps_position gps);
        float radians_(float degrees);
        gps_position radians_(gps_position gps);
        void set_acceleration_(coordinates_2d acceleration);
        void set_last_cartesian_(cart_pose pose);
        void set_last_bearing_(float bearing_mag);
        void set_last_yaw_vel_(float yaw_vel);
        void set_last_yaw_acc_(float yaw_acc);
    public:
        CartesianPose(gps_position magnetic_north, gps_position ref, coordinates_2d initial_vel, coordinates_2d initial_acc, float initial_bearing_mag);
        ~CartesianPose();
        cart_pose cartesian_pose(gps_position gps);
        cart_pose cartesian_pose(imu_data imu);
        cart_pose get_last_cartesian();
        float get_magnectic_declination();
        gps_position get_magnetic_north_gps();
        float get_last_bearing();
        gps_position get_last_gps();
        gps_position get_gps_ref();
        coordinates_2d get_acceleration();
        coordinates_2d get_velocity();
        float get_speed();
        float get_yaw_acceleration();
        float get_yaw_velocity();
        bool set_magnetic_declination(float declination);
        bool set_gps_ref(gps_position gps);
        bool set_last_gps(gps_position gps);
        bool set_magnetic_north_gps(gps_position gps);
        void set_velocity(coordinates_2d velocity);
};

#endif