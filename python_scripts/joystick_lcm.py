"""Script that polls a Logitech joystick and repeatedly transmits a 
corresponding controller command via LCM


TODO:
    Load a YAML file so that fewer arguments are required when running the
    file.
    Change the input based on the button pressed.

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

from joystick.joystick_controller import JoystickController
import lcm
from lcm_types.ControllerCommand import ControllerCommand 
import warnings
import argparse

# Ignore deprecation warnings from ``lcm`` dependencies
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Constants
DEFAULT_LCM_CHANNEL = "controller_input"  # Input controller message channel 
DEFAULT_DEVICE_ID = "/dev/input/js0"  # Default device ID for joystick device
DEFAULT_JOYSTICK_TYPE = "logitech"  # Default joystick type


def parse_args():
    """Parse input arguments"""
    parser = argparse.ArgumentParser(
        description='publish mode input messages via LCM.')
    parser.add_argument('channel', type=str, default=DEFAULT_LCM_CHANNEL,
                        help='LCM channel name to publish controller message on')
    parser.add_argument('device_id', type=str, default=DEFAULT_DEVICE_ID,
                        help='Joystick hardware device ID')
    parser.add_argument('-j', '--joystick-type', type=str,
                        default=DEFAULT_JOYSTICK_TYPE,
                        help=(f'Joystick type (`xbox` or `logitech`, default '
                              f'`{DEFAULT_JOYSTICK_TYPE}`)'))
    parser.add_argument('-lx', '--linear-scale-x', type=float, default=1.0,
                        help='Linear x-velocity scaling (default 1.0)')
    parser.add_argument('-ly', '--linear-scale-y', type=float, default=1.0,
                        help='Linear y-velocity scaling (default 1.0)')
    parser.add_argument('-lz', '--linear-scale-z', type=float, default=1.0,
                        help='Linear z-velocity scaling (default 1.0)')
    parser.add_argument('-ax', '--angular-scale-x', type=float, default=1.0,
                        help='Angular x-velocity scaling (default 1.0)')
    parser.add_argument('-ay', '--angular-scale-y', type=float, default=1.0,
                        help='Angular y-velocity scaling (default 1.0)')
    parser.add_argument('-az', '--angular-scale-z', type=float, default=1.0,
                        help='Angular z-velocity scaling (default 1.0)')
    args = parser.parse_args()
    return args


def publish_command(message, print_msg="", lcm_ch=DEFAULT_LCM_CHANNEL):
    """Publish an LCM ``ControllerCommand`` message on channel ``ch``, printing 
    ``print_msg`` to console, if provided."""

    # Publish Message
    lc = lcm.LCM()
    lc.publish(lcm_ch, message.encode())
    if print_msg:
        print(print_msg)


def header_str(hardware_id, lcm_channel):
    return '''
+--------------------------------------------------------------------+
| Polling for input from hardware device:  {:25s} |
| Publishing to channel:  {:42s} |
| Exit with Ctrl-C.                                                  |
+--------------------------------------------------------------------+
    '''.format(hardware_id, lcm_channel)


def main():
    # Parse input
    args = parse_args()
    device_id = args.device_id
    channel = args.channel

    # Initialize joystick
    p = JoystickController(
        device_id=device_id,
        joystick_type=args.joystick_type,
        linear_scaling=[args.linear_scale_x,
                        args.linear_scale_y, args.linear_scale_z],
        angular_scaling=[args.angular_scale_x,
                         args.angular_scale_y, args.angular_scale_z]
    )

    # Print header
    print(header_str(device_id, channel))

    # Loop
    try:
        while True:
            
            try:
                p.poll()
                publish_command(p.message, lcm_ch=channel)
            except ValueError:
                publish_command(ControllerCommand(), lcm_ch=channel)
    except KeyboardInterrupt:
        print('Keyboard interrupt.  Killing Robot and exiting.')
        for _ in range(5):  # ensure KILL message is received
            publish_command(ControllerCommand(), lcm_ch=channel)


if __name__ == '__main__':
    main()



