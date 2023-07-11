"""Contains class definition for Logitech F710 joystick.

This Files contains a specific class that contains mapping for Logitech F710.

TODO: 
    Add joystick model number(s).

Maintainer:
    Chandravaran Kunjeti (kunjeti@seas.upenn.edu)

License:
    BSD 3-Clause License

Copyright:
    Copyright 2022, The Trustees of the University of Pennsylvania. All Rights
    Reserved.
"""

from .joystick_abstract import Joystick

class LogitechJoystick(Joystick):
    """Joystick implementation for a Logitech F710 gamepad"""

    def __init__(self, *args, **kwargs):
        """Initialize joystick mappings and hardware interface."""

        print("Joystick mapping loaded for Logitech joystick.")
        super(LogitechJoystick, self).__init__(*args, **kwargs)
        
        # Axis and button names are assigned based on the mapping from the logitech
        self.axis_names = {
            0x00: 'left_stick_horz',
            0x01: 'left_stick_vert',
            0x03: 'right_stick_horz',
            0x04: 'right_stick_vert',

            0x02: 'L2_pressure',
            0x05: 'R2_pressure',

            0x10: 'dpad_leftright', # 1 is right, -1 is left
            0x11: 'dpad_up_down', # 1 is down, -1 is up

        }

        self.button_names = {
            0x13a: 'back',  
            0x13b: 'start',  
            0x13c: 'logitech',  

            0x130: 'A',
            0x131: 'B',
            0x133: 'X',
            0x134: 'Y',

            0x136: 'LB',
            0x137: 'RB',

            0x13d: 'left_stick_press',
            0x13e: 'right_stick_press',
        }

        super(LogitechJoystick, self).device_init()

    def run_loop(self):
        """Running the read operation in an infanite loop
        
        This function contains an infinite loop to run the polling and retrieving data.
        Can be used for debugging, but for real usuage advisable to use the run_once function
        """

        while True:
            button, button_state, axis, axis_val = self.poll()
            if axis is not None or button is not None:
                if button is None:
                    button = "0"
                    button_state = 0
                if axis is None:
                    axis = "0"
                    axis_val = 0
                message_data = (button, button_state, axis, axis_val)
                print(message_data)

    def run_once(self):
        """Running the read operation once

        This function samples retrieves the pool only once. It works like a trigger.
        """

        button, button_state, axis, axis_val = self.poll()
        if axis is not None or button is not None:
            if button is None:
                button = "0"
                button_state = 0
            if axis is None:
                axis = "0"
                axis_val = 0
            message_data = (button, button_state, axis, axis_val)
            print(message_data)
