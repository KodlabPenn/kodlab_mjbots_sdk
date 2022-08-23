#!/usr/bin/env python3

"""CLI for inputting modes via LCM.

This command line interface (CLI) populates an LCM ``ModeInput`` message, which
consists of a single signed integer, and sends that message on a designated LCM
channel.

Use this script as an example to build more complicated input CLIs.

Maintainer:
    Ethan J. Musser (emusser@seas.upenn.edu)

License:
    BSD 3-Clause License

Copyright:
    Copyright 2022, The Trustees of the University of Pennsylvania. All Rights
    Reserved.
"""

import lcm
from lcm_types.ModeInput import ModeInput
import warnings
import argparse

import os
import array
import time
import struct

# Ignore deprecation warnings from ``lcm`` dependencies
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Constants
Scalling = 32767.0

#def parse_args():
#    """Parse input arguments"""
#    parser = argparse.ArgumentParser(
#        description='publish mode input messages via LCM.')
#    parser.add_argument('channel', metavar='channel', type=str, nargs='+',
#                        help='LCM channel name to publish on')
#    args = parser.parse_args()
#    return args


class Joystick(object):
    '''
    Class used to interface to a physical joystick
    '''

    def __init__(self, dev_fn='/dev/input/js0'):
        self.axis_states = {}
        self.button_states = {}
        self.axis_map = []
        self.button_map = []
        self.jsdev = None
        self.dev_fn = dev_fn

    def init(self):
        try:
            from fcntl import ioctl
        except ModuleNotFoundError:
            self.num_axes = 0
            self.num_buttons = 0
            print("No support for fnctl module. joystick not enabled.")
            return

        if not os.path.exists(self.dev_fn):
            print(self.dev_fn, "is missing")
            return

        '''
        call once to setup connection to device and map buttons
        '''
        # Open the joystick device.
        print('Opening %s...' % self.dev_fn)
        self.jsdev = open(self.dev_fn, 'rb')

        # Get the device name.
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
        self.js_name = buf.tobytes().decode('utf-8')
        print('Device name: %s' % self.js_name)

        # Get number of axes.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf)  # JSIOCGAXES
        self.num_axes = buf[0]

        # Get number of buttons
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
        self.num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

        for axis in buf[:self.num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

        for btn in buf[:self.num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0
            # print('btn', '0x%03x' % btn, 'name', btn_name)

        return True

    def show_map(self):
        '''
        list the buttons and axis found on this joystick
        '''
        print('%d axes found: %s' % (self.num_axes, ', '.join(self.axis_map)))
        print('%d buttons found: %s' % (self.num_buttons, ', '.join(self.button_map)))

    def poll(self):
        '''
        query the state of the joystick, returns button which was pressed, if any,
        and axis which was moved, if any. button_state will be None, 1, or 0 if no changes,
        pressed, or released. axis_val will be a float from -1 to +1. button and axis will
        be the string label determined by the axis map in init.
        '''
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
                axis = self.axis_map[number]
                if axis:
                    fvalue = value / 32767.0
                    self.axis_states[axis] = fvalue
                    axis_val = fvalue

        return button, button_state, axis, axis_val

def publish_mode(mode, print_msg="", lcm_ch=DEFAULT_LCM_CHANNEL):
    """
    Publish an LCM ``ModeInput`` message with mode value ``mode`` on channel
    ``ch``.  Prints ``print_msg`` to console, if provided.
    """
    # Populate Message
    msg = ModeInput()
    assert isinstance(mode, int), 'Mode must be an integer.'
    print(f"Publishing Mode:  {mode}")
    msg.mode = mode

    # Publish Message
    lc = lcm.LCM()
    lc.publish(lcm_ch, msg.encode())
    if print_msg:
        print(print_msg)


def header_str(kill_int):
    return """
+--------------------------------------------------------------------+
| Enter a valid mode integer or 'q' to quit and return to OFF state. |
| Inputting mode %d or Ctrl-C will send a kill command.              |
+--------------------------------------------------------------------+
    """.format(kill_int)


def main():
    # Parse input
    channel = parse_args().channel[0]

    try:
        # Print header
        print(header_str(OFF))
        print(f"Publishing to channel \"{channel}\"")

        # Initially set mode to be ``OFF``
        mode = OFF

        # Repeatedly prompt for mode input
        while True:
            mode = input(f'Enter mode:  ')
            try:
                publish_mode(int(mode), lcm_ch=channel)
            except ValueError:
                if mode.upper() == 'Q':
                    publish_mode(OFF, 'Exiting.', lcm_ch=channel)
                    break
                else:
                    print('Invalid input.')
            except AssertionError:
                print('Invalid input.')
    except KeyboardInterrupt:
        print('Keyboard interrupt.  Killing Robot and exiting.')
        for _ in range(5):  # ensure KILL message is received
            publish_mode(KILL, lcm_ch=channel)


if __name__ == '__main__':
    main()
