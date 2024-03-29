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

# Ignore deprecation warnings from ``lcm`` dependencies
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Constants
OFF = 0  # OFF behavior integer
KILL = -1  # KILL behavior integer
DEFAULT_LCM_CHANNEL = "mode_input"


def parse_args():
    """Parse input arguments"""
    parser = argparse.ArgumentParser(
        description='publish mode input messages via LCM.')
    parser.add_argument('channel', metavar='channel', type=str, nargs='+',
                        help='LCM channel name to publish on')
    args = parser.parse_args()
    return args


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
