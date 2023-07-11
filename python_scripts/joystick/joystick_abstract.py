"""Provide an abstract class for interfacing with joystick hardware and an 
example implementation.

This file provides an abstract class with methods to interpret serial input 
from joystick devices. A sample child class implementation is also provided as
a guide for implementing custom joysticks.

This code was largely copied from [1]

Citations:
    [1] K. Kruempelstaedter, kevkruemp/donkeypart_logitech_controller, 2018. 
    [Online]. Available: https://github.com/kevkruemp/donkeypart_logitech_controller

Maintainer:
    Chandravaran Kunjeti (kunjeti@seas.upenn.edu)

License:
    BSD 3-Clause License

Copyright:
    Copyright 2022, The Trustees of the University of Pennsylvania. All Rights
    Reserved.
"""

import os
import array
import time
import struct
from fcntl import ioctl


INT16_MAX = 32767

class Joystick(object):
    """Abstract class defining different joystick hardware.
    
    This abstract class contains functions to interpret the serial input from
    joystick devices. Based on the mapping that will be setup by the inherited 
    class the joystick values can be mapped properly. The functions implemented 
    in the class can be used for joystick device.
    """

    def __init__(self, device_id='/dev/input/js0'):
        self.axis_states = {} # Contains the mapping of the axis for joystick
        self.button_states = {} # Contains the mapping of the buttons for joystick
        self.axis_list = [] 
        self.button_map = []
        self.jsdev = None # Variable to store the open device port.
        self.set_device_id(device_id) 
        self.axis_normalization = INT16_MAX # Sets the joystick ADC range 
 
    def device_init(self):
        """Setup connection to device and mapping of buttons.
        
        Initialization method that opens the device port, reads the 
        number of axes and buttons, and maps them. Contains a single call to
        setup connection to device and mapping of buttons.

        Note:
            This init needs to called after the axis_names and button_names are 
            defined.
        """

        try:
            from fcntl import ioctl
        except ModuleNotFoundError:
            self.num_axes = 0
            self.num_buttons = 0
            print("no support for fnctl module. joystick not enabled.")
            return

        if not os.path.exists(self.device_id):
            print("Joystick device", self.device_id, "is missing")
            return

        # Open the joystick device.
        print('Opening %s...' % self.device_id)
        self.jsdev = open(self.device_id, 'rb')

        # Get the device name.
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
        self.js_name = buf.tobytes().decode('utf-8')
        print('Device name: %s' % self.js_name)

        # Get number of axes.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf)  # JSIOCGAXES
        self.num_axes = buf[0]

        # Get number of buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
        self.num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

        for axis in buf[:self.num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_list.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

        for btn in buf[:self.num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0
            # print('btn', '0x%03x' % btn, 'name', btn_name)
            # TODO: Set a input variable to get

        return True

    def show_map(self):
        """Print joystick axis and button mappings.

        List the buttons and axis found on this joystick.
        """
        print('%d axes found: %s' % (self.num_axes, ', '.join(self.axis_list)))
        print('%d buttons found: %s' % (self.num_buttons, ', '.join(self.button_map)))

    def set_device_id(self, device_id):
        """Sets device id

        Function allowing us to set the device_id.
        """
        self.device_id = device_id

    def poll(self):
        """Read the joystick port

        Query the state of the joystick, returns button which was pressed,
        if any, and axis which was moved, if any. button_state will be None,
        1, or 0 if no changes, pressed, or released. axis_val will be a 
        float from -1 to +1. button and axis will be the string label
        determined by the axis map in init.

        Returns:
            The button name, button value, axis name, axis value. These are 
            returned as a seperate value. 
        """

        button = None
        button_state = None
        axis = None
        axis_val = None

        if self.jsdev is None:
            return button, button_state, axis, axis_val

        # Main event loop
        evbuf = self.jsdev.read(8)

        if evbuf:
            tval, value, typev, number = struct.unpack('IhBB', evbuf)

            if typev & 0x80:
                # ignore initialization event
                return button, button_state, axis, axis_val

            if typev & 0x01:
                button = self.button_map[number]
                # print(tval, value, typev, number, button, 'pressed')
                if button:
                    self.button_states[button] = value
                    button_state = value

            if typev & 0x02:
                axis = self.axis_list[number]
                if axis:
                    fvalue = value / self.axis_normalization
                    self.axis_states[axis] = fvalue
                    axis_val = fvalue

        return button, button_state, axis, axis_val

class JoystickCreator(Joystick):
    """Example joystick implementation class
    
    A sample class that user-defined joysticks should be modelled off of. The 
    axis button names, which are specific to the joystick being used, are 
    loaded here.

    Anyone using this class as an example should populate the 
    ``self.axis_names`` and ``self.button_names`` dictionaries with their 
    desired mappings.
    """

    def __init__(self, *args, **kwargs):
        super(JoystickCreator, self).__init__(*args, **kwargs)

        self.axis_names = {}
        self.button_names = {}

    def run(self):
        while True:
            button, button_state, axis, axis_val = self.poll()
            print(button)
            if axis is not None or button is not None:
                if button is None:
                    button = "0"
                    button_state = 0
                if axis is None:
                    axis = "0"
                    axis_val = 0
                message_data = (button, button_state, axis, axis_val)
                print(message_data)

