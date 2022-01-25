import lcm
from lcm_types.TVHLog import TVHLog
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import argparse
from copy import deepcopy

def add_highlight(axis, timestamp_in,hybrid_modes_in):
    timestamps = deepcopy(timestamp_in)
    hybrid_modes = deepcopy(hybrid_modes_in)
    ylim = axis.get_ylim()

    start_idx = 0
    end_idx = 0
    while True:
        try:
            start_idx = hybrid_modes.index(2, end_idx + 1)
            end_idx = hybrid_modes.index(1,start_idx + 1)
            start_time = timestamps[start_idx]
            length = timestamps[end_idx]-start_time
            rect = patches.Rectangle((start_time, ylim[0]), length, ylim[1]-ylim[0], linewidth=1, edgecolor='none', facecolor='silver')
            axis.add_patch(rect)
        except ValueError:
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='create plot from log.')
    parser.add_argument('log', metavar='L', type=str, nargs='+',
                        help='log file name')
    args = parser.parse_args()
    file_name = 'logs/' + args.log[0]
    print(file_name)

    log = lcm.EventLog(file_name, "r")

    timestamps = []
    margins = []
    hybrid_mode = []
    tail_angle = []
    tail_torque = []
    leg_comp = []
    leg_speed = []
    for event in log:
        if event.channel == "jerboa_data":
            msg = TVHLog.decode(event.data)
            timestamps.append(msg.timestamp)
            margins.append(msg.margin)
            hybrid_mode.append(msg.hybrid_mode)
            tail_angle.append(msg.tail_angle)
            tail_torque.append(msg.torque_cmd)
            leg_comp.append(msg.leg_comp)

    timestamps = np.array(timestamps)
    margins = np.array(margins)

    mean_dt = np.average(np.diff(timestamps))
    mean_margin = np.average(margins)
    print("Mean dt = ", mean_dt)
    print("stdev dt = ", np.std(np.diff(timestamps)))

    print("Mean margin = ", mean_margin)
    print("std margin = ", np.std(margins))

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)
    ax1.set_title(file_name)
    ax1.plot(timestamps, hybrid_mode)

    ax1.set_ylabel('hybrid mode')

    ax2.plot(timestamps, leg_comp)
    ax2.set_ylabel('leg compression (m)')

    ax3.plot(timestamps, tail_angle)
    ax3.set_ylabel('Tail angle (rad)')

    ax4.plot(timestamps, tail_torque)
    ax4.set_ylabel('tail torque (Nm)')

    add_highlight(ax1, timestamps, hybrid_mode)
    add_highlight(ax2, timestamps, hybrid_mode)
    add_highlight(ax3, timestamps, hybrid_mode)
    add_highlight(ax4, timestamps, hybrid_mode)
    plt.show()