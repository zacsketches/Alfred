#Alfred

Various implementations of my Glow-Worm framework on a dagu Rover5 
platform.

Each variation is a different branch in the repo.  Complete builds
are merged into the master branch and can be loaded via their .ino
files in the folders included here.

##helen_keller:

This version wanders around and reacts to bumping into things.  It
is equipped with a set of bumpers printed on our 3D printer.  Files
for the bumpers are in the prints folder.  Each bumper is wired with
two mementary pushbutton switches which grounds the input pin and
allows the robot to react to the bump.  Reaction time can be set
in the by calling .set_maneuver_time(int) on the helen_controller
instance before calling the begin() function.
