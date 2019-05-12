//
// Created by davidm on 5/10/19.
//

#include "awsp_controller/pd_controller.h"


PDController::PDController(double dt, double max_output, double min_output)
{
    dt_ = dt;
    max_output_ = max_output;
    min_output_ = min_output;
    previous_error_ = 0;
};
PDController::~PDController(){};

double PDController::get_force(double error, double p_gain, double d_gain)
{
    double p_out = p_gain * error;
    double derivative = (error - previous_error_) / dt_;
    double d_out = d_gain * derivative;

    double output = p_out + d_out;

    if(output > max_output_)
        output = max_output_;
    else if(output < min_output_)
        output = min_output_;

    previous_error_ = error;

    return output;
}


