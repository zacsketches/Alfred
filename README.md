#Alfred

My Glow Worm Arduino based robotics framework on a Dagu Rover5 
platform.

The 

##Tags

 - v1.0 - basic functionality with just the bump sensors and the
helen keller controller.
 - v1.1 - Stil manuevering with just the bump sensors, but this 
version adds the light plant for a visual indication of what was 
bumped.
 - v2.0 - Wandering with the five pt scanner.
 - v2.1 - Wandering with the five pt scanner and the light plant.

##Development control systems

 - helen_keller - This version wanders around and reacts to bumping 
into things.  It is equipped with a set of bumpers printed on our 
3D printer.  Files for the bumpers are in the prints folder.  Each 
bumper is wired with two mementary pushbutton switches which grounds 
the input pin and allows the robot to react to the bump.  Reaction 
time can be set by calling `.set_maneuver_time(int)` on the 
helen_controller instance before calling the `.begin()` function.

- whisker - This version wanders too, but uses an ultrasonic 
sensor to attempt obstacle avoidance.  The sensor is good at detecting
walls and other obstacles, but has trouble with skinny chair legs
or other thin obstacles.  The header file `scanner.h` contains the
`Scanner_5pt` class which hides the processing necessary to 
control the sonar sensor and the servo it is mounted to. 
