#include "catamaran_controller/esc_lib.h"

esc_lib::esc_lib(int pin){
    pin_ = pin;
	setup_();
}

esc_lib::~esc_lib(){}
		
void esc_lib::setup_(){
	std::cout << "Starting the pigpio daemon." << std::endl;	
	system("sudo pigpiod");
	sleep(2);

	pi_ = pigpio_start(NULL, NULL);

	// This checks if the pigpio daemon is running
	if(pi_ < 0)
	{
		std::cout << "Failed.... Daemon is NOT running." << std::endl;
		exit(0);
	}
	else
	{
		std::cout << "Daemon was succesfully initialized." << std::endl;
	}

	// This checks if the ESC/motor has already been initialised
	if(!get_mode(pi_, pin_))
	{
		std::cout << "ERROR: Something is preventing the motor from initializing." << std::endl;
		std::cout << "SUGGESTION: Check that the correct pin was chosen." << std::endl;
		std::cout << "SUGGESTION: Check if the motor needs throttle stick calibration." << std::endl;
		end();
		exit(0);
	}
	else
	{
		set_servo_pulsewidth(pi_, pin_, NEUTRAL);
		std::cout << "Motor was succesfully initialized" << std::endl;
	}

}

void esc_lib::setSpeed(int speed){
	//std::cout << "speed: " << speed << std::endl;
	set_servo_pulsewidth(pi_, pin_, speed);
}

void esc_lib::end(){
	std::cout <<  std::endl << "Motor initialization terminated." << std::endl << std::endl;
	set_servo_pulsewidth(pi_, pin_, NEUTRAL);
	pigpio_stop(pi_);
	stop_thread(NULL);
	system("sudo killall pigpiod");
}
