"""Provides the ``JoystickController`` aggregate joystick hardware class.

TODO:
    Need to load a YAML file so that many arguments are not required 
    to be passed when we start the file.

Maintainer:
    Chandravaran Kunjeti (kunjeti@seas.upenn.edu)

Author:
    Chandravaran Kunjeti (kunjeti@seas.upenn.edu)
    Ethan J. Musser (emusser@seas.upenn.edu)

License:
    BSD 3-Clause License

Copyright:
    Copyright 2022, The Trustees of the University of Pennsylvania. All Rights
    Reserved.
"""

from .joystick_logitech import LogitechJoystick
from .joystick_xbox import XboxJoystick
from lcm_types.ControllerCommand import ControllerCommand


class JoystickController(object):
    """Class accepting multiple joysticks and selecting one based on user 
    specification.

    This class combines the hardware controllers for the Logitech and X-Box 360 
    joysticks. The proper controller is loaded based on construction 
    parameters.
    """

    def __init__(self, linear_scaling=[1.0, 1.0, 1.0],
                 angular_scaling=[1.0, 1.0, 1.0],
                 joystick_type='logitech', device_id='/dev/input/js0'):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.message = ControllerCommand()
        self.message_map = {}
        self.device_id = device_id
        self.joystick_select(joystick_type)


    def joystick_select(self, joystick_type):
        """Select a joystick given a joystick name."""

        self.joystick_type = joystick_type
        if joystick_type == "logitech":
            self.c = LogitechJoystick(device_id=self.device_id)
            self.message_map['linear_x'] = self.c.axis_names.get(0x01)
            self.message_map['linear_y'] = self.c.axis_names.get(0x00)
            self.message_map['linear_z'] = self.c.axis_names.get(0x11)
            self.message_map['angular_x'] = self.c.axis_names.get(0x10)
            self.message_map['angular_y'] = self.c.axis_names.get(0x04)
            self.message_map['angular_z'] = self.c.axis_names.get(0x03)
        elif joystick_type == "xbox":
            self.c = XboxJoystick(device_id=self.device_id)
            self.message_map['linear_x'] = self.c.axis_names.get(0x01)
            self.message_map['linear_y'] = self.c.axis_names.get(0x00)
            self.message_map['linear_z'] = self.c.axis_names.get(0x11)
            self.message_map['angular_x'] = self.c.axis_names.get(0x10)
            self.message_map['angular_y'] = self.c.axis_names.get(0x04)
            self.message_map['angular_z'] = self.c.axis_names.get(0x03)
        else:
            print(f'[WARN] Invalid joystick selected, `{joystick_type}` is not'
                  f'a valid joystick.')

    def poll(self):
        """Wrapper Function for poll.
        
        The Abstract class poll function is called, These values are scaled 
        based on the factor we require. The scaling is set by the user. 
        
        TODO: 
            Add a YAML file to load the parameters.
        """

        button, button_state, axis, axis_val = self.c.poll()
        
        if axis == self.message_map['linear_x']:
            self.message.linear[0] = axis_val * self.linear_scaling[0]
        elif axis == self.message_map['linear_y']:
            self.message.linear[1] = axis_val * self.linear_scaling[1]
        elif axis == self.message_map['linear_z']:
            self.message.linear[2] = axis_val * self.linear_scaling[2]
        elif axis == self.message_map['angular_x']:
            self.message.angular[0] = axis_val * self.angular_scaling[0]
        elif axis == self.message_map['angular_y']:
            self.message.angular[1] = axis_val * self.angular_scaling[1]
        elif axis == self.message_map['angular_z']:
            self.message.angular[2] = axis_val * self.angular_scaling[2]   

