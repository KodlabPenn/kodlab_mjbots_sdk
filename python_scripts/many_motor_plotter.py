import lcm
from lcm_types.many_motor_log import many_motor_log
import matplotlib.pyplot as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='create plot from log.')
parser.add_argument('log', metavar='L', type=str, nargs='+',
                    help='log file name')
args = parser.parse_args()
file_name = 'logs/'+args.log[0]
print(file_name)

log = lcm.EventLog(file_name, "r")


timestamps = []
margins = []
positions = []
velocities = []
torques = []
for event in log:
    if event.channel == "motor_data":
        msg = many_motor_log.decode(event.data)
        timestamps.append(msg.timestamp)
        velocities.append(msg.velocities)
        positions.append(msg.positions)
        margins.append(msg.mean_margin)
        torques.append(msg.torques)


timestamps = np.array(timestamps)
margins = np.array(margins)
velocities = np.array(velocities)
positions = np.array(positions)

mean_dt = np.average(np.diff(timestamps))
mean_margin = np.average(margins)
print("Mean dt = ",mean_dt)
print("stdev dt = ",np.std(np.diff(timestamps)))

print("Mean margin = ", mean_margin)
print("std margin = ", np.std(margins))

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
ax1.set_title(file_name)
ax1.plot(timestamps, margins)
ax1.set_ylabel('Communication duration')

ax2.plot(timestamps[:-1], np.diff(timestamps))
ax2.set_ylabel('dt(ms)')

ax3.plot(timestamps, torques)
ax3.set_ylabel('Torque(Nm)')

ax4.plot(timestamps, positions)
ax4.set_ylabel('Positions(rad)')

plt.show()


