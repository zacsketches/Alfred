#ifndef CONTROL_VEC_H
#define CONTROL_VEC_H JUN_2014

#include <clearinghouse.h>
#include <messages/cmd_velocity.h>

//*******************************************************************
//*                         CONTROL_VEC
//* Struct to hold a control command for the motors
//*******************************************************************

namespace Control {
    /*------Motor Control Constants-----------------*/
    //                                                  
															//left mtr                      //right mtr
    const Cmd_velocity_msg forward_easy("forward_easy", Direction::fwd,   70,           Direction::fwd,   70);
    const Cmd_velocity_msg backtwist_CCW("back_CCW",     Direction::bck,   70,           Direction::bck,   20);
    const Cmd_velocity_msg backtwist_CW("back_CW",      Direction::fwd,   20,           Direction::bck,   70);

    const int number_of_controls = 9;
} //namespace control

#endif
