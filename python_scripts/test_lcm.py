import lcm
from lcm_types.motor_log import motor_log
import matplotlib.pyplot as plt
import numpy as np

log = lcm.EventLog('logs/lcmlog-2021-11-10.14', "r")


timestamps = []
margins = []
positions = []
velocities = []
for event in log:
    if event.channel == "EXAMPLE":
        msg = motor_log.decode(event.data)
        timestamps.append(msg.timestamp)
        velocities.append(msg.velocities)
        positions.append(msg.positions)
        margins.append(msg.mean_margin)

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
ax1.plot(timestamps, margins)
ax1.set_ylabel('margins')

ax2.plot(timestamps, positions)
ax2.set_ylabel('positions')

ax3.plot(timestamps, velocities)
ax3.set_ylabel('velocities')

ax4.plot(timestamps[:-1], np.diff(timestamps))
ax4.set_ylabel('dt(ms)')

plt.show()


