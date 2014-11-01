#ifndef CONTROL_VECTORS_H
#define CONTROL_VECTORS_H SEP_2014

#include <clearinghouse.h>
#include <messages/cmd_velocity.h>

namespace Control {
    /*------Motor Control Constants-----------------*/
    //                                                  
    const Cmd_velocity_msg forward       ("forwd", Direction::fwd, 100, Direction::fwd, 100);
    const Cmd_velocity_msg forward_easy  ("f_eas", Direction::fwd,  70, Direction::fwd,  70);
    const Cmd_velocity_msg back          ("back ", Direction::bck,  60, Direction::bck,  60);
    const Cmd_velocity_msg back_twist_cw ("b__cw", Direction::bck,  20, Direction::bck, 100);
    const Cmd_velocity_msg back_twist_ccw("b_ccw", Direction::bck, 125, Direction::bck,  10);
    const Cmd_velocity_msg ease_right    ("ea_rt", Direction::fwd, 100, Direction::fwd,  40);
    const Cmd_velocity_msg ease_left     ("ea_lt", Direction::fwd,  50, Direction::fwd, 100);
    const Cmd_velocity_msg swerve_right  ("sw_rt", Direction::fwd, 115, Direction::bck,  55);
    const Cmd_velocity_msg swerve_left   ("sw_lt", Direction::bck,  50, Direction::fwd, 100);
    const Cmd_velocity_msg twist_right   ("tw_rt", Direction::fwd, 100, Direction::bck, 110);

    const int number_of_controls = 10;
} //namespace control

#endif
