//
// Created by davidm on 5/10/19.
//

#include "awsp_controller/pd_controller.h"


PDController::PDController(double dt, double max_output,
        double min_output, double p_gain, double d_gain)
{
    dt_ = dt;
    max_output_ = max_output;
    min_output_ = min_output;
    p_gain_ = p_gain;
    d_gain_ = d_gain;
    previous_error_ = 0;
};
PDController::~PDController(){};

double PDController::get_force(double set_point, double pv)
{
    double error = set_point - pv;
    double p_out = p_gain_ * error;
    double derivative = (error - previous_error_) / dt_;
    double d_out = d_gain_ * derivative;

    double output = p_out + d_out;

    if(output > max_output_)
        output = max_output_;
    else if(output < min_output_)
        output = min_output_;

    previous_error_ = error;

    return output;
}


