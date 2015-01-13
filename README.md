#Alfred

My Glow Worm Arduino based robotics framework on a Dagu Rover5 
platform.

The Alfred_master file contains the most advanced sketch that 
integrates the ultrasonic sensor with the bump sensors and
a light plant to provide some feedback on where the robot is
sensing obstructions.

Unfortunately **AND THIS IS BIG** I had to change the buffer size
wire.h, twi.h, and hardwareSerial.h located in the Arduino libraries
in order to create enough room for this code to load on Arduino Uno.

If you aren't up for changing your libraries (using git to change 
between original/altered version) you have two choices.  Load on the 
Mega or Due or use one of the small sketches in the dev_sketches
folder.

All these sketches use the tank tread style drive system that comes
stock on the Dagu 5.  The control system in all these robots picks
a control vector from the `control_vec.h` file.  To tune the
robot's response tweak the values in these control vectors.

Most sketch folders also include a `XXX_controller.h` file that
implements the decision logic for choosing a control vector based
on the results of the sensor input, whether that is the ultrasonic
sensor or the bump sensors. 

##Development control systems

These sketches are located in the dev_sketches folder

 - Helen_bot - This version wanders around and reacts to bumping 
into things (Not that politically correct of a name, but hey..she
was freaked out in her later years and became an ardent support of 
Stalin.  She had it coming).  The Helen_bot is equipped with a set 
of bumpers printed on our 3D printer.  Files for the bumpers are 
in the prints folder.  Each bumper is wired with two momentary 
pushbutton switches which ground the input pin and allows the 
robot to react to the bump.  Each time a pin is grounded (because a 
bumper ran into something) the robot will back away from the
obstruction for a proscribed reaction time.  This reaction time can 
be set by calling `.set_maneuver_time(int)` on the helen_controller 
instance before calling the `.begin()` function.

- Whisker_bot - This version wanders too, but uses an ultrasonic 
sensor to attempt obstacle avoidance.  The sensor is good at detecting
walls and other obstacles, but has trouble with skinny chair legs
or other thin obstacles.  The header file `scanner.h` contains the
`Scanner_5pt` class which hides the processing necessary to 
control the sonar sensor and the servo it is mounted to as a scanner
object.

- Light_whisker_bot - This version doesn't change the control system
from the Whisker_bot.  However it adds feedback by incorporating a
bank of 5 LEDs to show which 'whisker' is getting obstruction 
feedback.  I still need to add the schematic for this light plant and
I will probably use our 3D printer to produce a new one.

- Message_test - This sketch demonstrates the print capabilities of
some Glow Worm Framework components.  Note that detailed printing 
from `GW::node`s is disabled by default.  This limits the memory
consumption for each sketch.  To turn printing on use the `#define`
statements at the top of each block.h file.

 
