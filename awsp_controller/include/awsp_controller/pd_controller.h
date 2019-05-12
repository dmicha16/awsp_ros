//
// Created by davidm on 5/10/19.
//

#ifndef PROJECT_PD_CONTROLLER_H
#define PROJECT_PD_CONTROLLER_H

class PDController
{
public:
    PDController(double dt, double max_output, double min_output);
    ~PDController();

    double get_force(double error, double p_gain, double d_gain);

private:
    double dt_;
    double max_output_;
    double min_output_;
    double previous_error_;
    double force_;

};

#endif //PROJECT_PD_CONTROLLER_H
