from statistics import mode

import lcm
import matplotlib

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


def max_compressions(compression,hybrid_modes):
    end_idx = 0
    max_compression_list = []
    while True:
        try:
            start_idx = hybrid_modes.index(2, end_idx + 1)
            end_idx = hybrid_modes.index(1,start_idx + 1)
            max_compression_list.append(max(compression[start_idx:stop_idx]))
        except ValueError:
            break
    return max_compression_list


def mean_max_compression(compression,hybrid_modes):
    return np.average(np.array(max_compressions(compression,hybrid_modes)))


def rindex(lst, value):
    lst.reverse()
    i = lst.index(value)
    lst.reverse()
    return len(lst) - i - 1


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
    tail_torque_measured = []
    leg_comp = []
    leg_speed = []
    tail_speed = []
    mass_tail = []
    mass = []
    kv = []
    k = []
    for event in log:
        if event.channel == "jerboa_data":
            msg = TVHLog.decode(event.data)
            timestamps.append(msg.timestamp)
            margins.append(msg.margin)
            hybrid_mode.append(msg.hybrid_mode)
            tail_angle.append(msg.tail_angle)
            tail_torque.append(msg.torque_cmd)
            leg_comp.append(msg.leg_comp)
            leg_speed.append(msg.leg_speed)
            tail_speed.append(msg.tail_speed)
            tail_torque_measured.append(msg.torque_measured)
            k.append(msg.k)
            kv.append(msg.kv)
            mass.append(msg.m)
            mass_tail.append(msg.mt)

    timestamps = np.array(timestamps)
    margins = np.array(margins)

    mean_dt = np.average(np.diff(timestamps))
    mean_margin = np.average(margins)
    print("Mean dt = ", mean_dt)
    # print("stdev dt = ", np.std(np.diff(timestamps)))

    print("Mean margin = ", mean_margin)
    # print("std margin = ", np.std(margins))

    touchdown_idx = hybrid_mode.index(2)
    liftoff_idx = rindex(hybrid_mode, 2)

    start_idx = max(touchdown_idx-100, 0)
    stop_idx = min(liftoff_idx+100, len(timestamps)-1)

    timestamps = timestamps[start_idx:stop_idx]
    margins = margins[start_idx:stop_idx]
    hybrid_mode = hybrid_mode[start_idx:stop_idx]
    tail_angle = tail_angle[start_idx:stop_idx]
    tail_torque = tail_torque[start_idx:stop_idx]
    leg_comp = leg_comp[start_idx:stop_idx]
    leg_speed = leg_speed[start_idx:stop_idx]
    tail_speed = tail_speed[start_idx:stop_idx]
    tail_torque_measured = tail_torque_measured[start_idx:stop_idx]
    time_since_td = timestamps/1000-timestamps[0]/1000

    mass_tail = mass_tail[start_idx:stop_idx]
    mass = mass[start_idx:stop_idx]
    kv = kv[start_idx:stop_idx]
    k = k[start_idx:stop_idx]

    k = mode(k)
    kv = mode(kv)
    mass = mode(mass)
    mass_tail = mode(mass_tail)
    mean_bottom = mean_max_compression(leg_comp, hybrid_mode)
    apex_height = ((1/2 * k * mean_bottom ** 2)/mass/9.81 - mean_bottom) * 100
    apex_energy = 1/2 * k * mean_bottom ** 2

    color1 = 'tab:blue'
    color2 = 'tab:red'

    font = {'family' : 'Dejavu Sans',
            'weight' : 'normal',
            'size'   : 14}

    matplotlib.rc('font', **font)

    fig, (ax2, ax3, ax4) = plt.subplots(3, 1, sharex=True)

    ax2.plot(time_since_td, leg_comp, color=color1)
    ax2.set_ylabel('leg compression (m)', color=color1)
    ax2.tick_params(axis='y', labelcolor=color1)

    ax2l = ax2.twinx()
    ax2l.plot(time_since_td, leg_speed, color=color2)
    ax2l.set_ylabel('leg speed (m/s)', color=color2)
    ax2l.tick_params(axis='y', labelcolor=color2)

    ax2.set_xlabel('time (s)')
    ax2.set_zorder(1)  # default zorder is 0 for ax1 and ax2
    ax2.patch.set_visible(False)  # prevents ax1 from hiding ax2

    ax3.plot(time_since_td, tail_angle, color=color1)
    ax3.set_ylabel('Tail angle (rad)', color=color1)
    ax3.tick_params(axis='y', labelcolor=color1)

    ax3l = ax3.twinx()
    ax3l.plot(time_since_td, tail_speed, color=color2)
    ax3l.set_ylabel('tail speed (rad/s)', color=color2)
    ax3l.tick_params(axis='y', labelcolor=color2)

    ax3.set_xlabel('time (s)')
    ax3.set_zorder(1)  # default zorder is 0 for ax1 and ax2
    ax3.patch.set_visible(False)  # prevents ax1 from hiding ax2

    ax3.set_zorder(2)  # default zorder is 0 for ax1 and ax2
    ax3.patch.set_visible(False)  # prevents ax1 from hiding ax2

    ax4.plot(time_since_td, tail_torque_measured, color=color2)
    ax4.plot(time_since_td, tail_torque, color=color1)
    ax4.set_ylabel('tail torque (Nm)', color=color1)
    ax4.tick_params(axis='y', labelcolor=color1)
    ax4.set_xlabel('time (s)')

    ax2.grid()
    ax3.grid()
    ax4.grid()
    add_highlight(ax2l, time_since_td, hybrid_mode)
    add_highlight(ax3l, time_since_td, hybrid_mode)
    add_highlight(ax4, time_since_td, hybrid_mode)

    fig.suptitle("kv = " + str(kv) + ", Height = " + str(np.round(apex_energy, 2)) + "J, k = " + str(np.round(k)) + "N/m, mt = " + str(np.round(mass_tail,3)) + "kg")


    # fig, ax = plt.subplots()
    #
    # ax.plot(leg_comp, leg_speed)
    # ax.set_xlabel('Leg compression (m)')
    # ax.set_ylabel('Leg speed (m/s)')
    # ax.grid()
    plt.show()