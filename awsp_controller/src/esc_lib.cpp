#include "awsp_controller/esc_lib.h"

esc_lib::esc_lib(int pin)
{
    pin_ = pin;
}

esc_lib::~esc_lib() {}
		
bool esc_lib::setup(){
	ROS_WARN("===== BEGAN PIGPIO DEAMON INITIALIZATION. =====");
	system("sudo pigpiod");
	sleep(2);

	pi_ = pigpio_start(NULL, NULL);

	// This checks if the pigpio daemon is running
	if(pi_ < 0)
	{
//		ROS_ERROR("DEAMON FAILED TO INITIALIZE.");
//		ROS_ERROR("RE-INITIALIZATION IS REQUIRED TO MOVE THE BOAT.");
		return false;
	}
	else
	{
//		ROS_INFO("DAEMON SUCCESSFULLY INITIALIZED");
		// This checks if the ESC/motor has already been initialised
		if(!get_mode(pi_, pin_))
		{
			ROS_ERROR("MOTOR CANNOT BE INITIALIZED AT THIS TIME.");
//			ROS_INFO("SUGGESTION: Check that the correct pin was chosen.");
//			ROS_INFO("SUGGESTION: Check if the motor needs throttle stick calibration.");
			return false;
		}
		else
		{
			set_servo_pulsewidth(pi_, pin_, NEUTRAL);
			ROS_INFO("===== MOTOR WAS INITIALIZED. =====");
			return true;
		}
	}


}

void esc_lib::setSpeed(int speed)
{
	//std::cout << "speed: " << speed << std::endl;
	set_servo_pulsewidth(pi_, pin_, speed);
}

void esc_lib::end()
{
	std::cout <<  std::endl << "Motor initialization terminated." << std::endl << std::endl;
	set_servo_pulsewidth(pi_, pin_, NEUTRAL);
	pigpio_stop(pi_);
	stop_thread(NULL);
	system("sudo killall pigpiod");
}
