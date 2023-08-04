# Overview
The kodlab_mjbots_sdk has a few key key features added to the 
pi3_hat library developed by mjbots: https://github.com/mjbots/pi3hat
1. Cross compiling support from ubuntu 20.04 to Raspberry pi
2. Integration with NYU realtime_tools library: https://github.com/machines-in-motion/real_time_tools 
for better realtime performance
3. Integration with LCM (https://lcm-proj.github.io/) for remote logging and remote input to the robot
4.  The `MjbotsControlLoop` object which handles the structure of the control loop for
the easy creation of new controllers
5. The `MjbotsHardwareInterface` which provides a convenient interface for communicating with any number
of moteus motor controllers 
6. The `RobotBase` class which provides an interface for updating robot state and joint torques.

Note: This library only supports torque commands. If you wish to use
position control, you either must close the loop yourself or modify the 
library to allow for the position loop to Run on the moteus.

# Citation
To cite this repo please use the information in the CITATION.cff file

# Acknowledgement
This work was supported by ONR grant #N00014- 16-1-2817, a Vannevar Bush Fellowship held by Daniel Koditschek,
sponsored by the Basic Research Office of the Assistant Secretary of Defense for Research and Engineering.
