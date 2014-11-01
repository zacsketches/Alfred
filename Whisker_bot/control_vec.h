#include <Direction.h>

#ifndef CONTROL_VEC_H
#define CONTROL_VEC_H JUN_2014

//*******************************************************************
//*                         CONTROL_VEC
//* Struct to hold a control command for the motors
//*******************************************************************
struct Control_vec{
    char label[15];
    Direction::dir l_dir;
    int l_pwm;
    Direction::dir r_dir;
    int r_pwm;
};

namespace Control {
    /*------Motor Control Constants-----------------*/
    //                                                  
                                                      //left mtr                      //right mtr
    const Control_vec forward =      {"forward",      Direction::fwd,   70,           Direction::fwd,   70};
    const Control_vec forward_easy=  {"forward_easy", Direction::fwd,   70,           Direction::fwd,   70};
    const Control_vec backward =     {"backward",     Direction::back,  60,           Direction::back,  60};
    const Control_vec backward_twist={"back_twist",   Direction::back,  60,           Direction::back,  80};
    const Control_vec ease_right =   {"ease_right",   Direction::fwd,  100,           Direction::fwd,   40};
    const Control_vec ease_left =    {"ease_left",    Direction::fwd,   50,           Direction::fwd,  100};
    const Control_vec swerve_right = {"swerve_right", Direction::fwd,  115,           Direction::back,  55};
    const Control_vec swerve_left =  {"swerve_left",  Direction::back,  50,           Direction::fwd,  100};
    const Control_vec twist_right =  {"twist_right",  Direction::fwd,  100,           Direction::back, 110};

    const int number_of_controls = 9;
} //namespace control

#endif
