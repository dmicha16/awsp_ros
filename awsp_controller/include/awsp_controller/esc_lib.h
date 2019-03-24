#ifndef ESC_LIB_H
#define ESC_LIB_H

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <pigpiod_if2.h>


class esc_lib
{
    private:
        int pin_ = 0;
        int pi_ = -1;
	    void setup_();

    public:
        esc_lib(int pin);
        ~esc_lib();

        const int NEUTRAL = 1500;
        void setSpeed(int speed);
        void end();

};

#endif