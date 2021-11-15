import lcm
from lcm_types.leg_log import leg_log
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
r = []
theta = []
dr = []
dtheta = []
for event in log:
    if event.channel == "EXAMPLE":
        msg = leg_log.decode(event.data)
        timestamps.append(msg.timestamp)
        velocities.append(msg.velocities)
        positions.append(msg.positions)
        margins.append(msg.mean_margin)
        torques.append(msg.torque_cmd)
        r.append(msg.polar_position[0])
        theta.append(msg.polar_position[1])
        dr.append(msg.polar_vel[0])
        dtheta.append(msg.polar_vel[1])


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
print(np.mean(positions[:,0]))
print(np.mean(positions[:,1]))

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
ax1.set_title(file_name)
ax1.plot(timestamps, margins)
ax1.set_ylabel('margins')

ax2.plot(timestamps[:-1], np.diff(timestamps))
ax2.set_ylabel('dt(ms)')

ax3.plot(timestamps, torques)
ax3.set_ylabel('Torque(Nm)')

ax4.plot(timestamps, positions)
ax4.set_ylabel('Positions(rad)')

fig2, (fig2_ax1, fig2_ax2) = plt.subplots(2, 1)
fig2_ax1.set_title(file_name)
fig2_ax1.plot(timestamps, r)
fig2_ax1.set_ylabel('leg length')

fig2_ax2.plot(timestamps, theta)
fig2_ax2.set_ylabel('theta(rad)')


plt.show()


