import lcm
from lcm_types.ManyMotorLog import ManyMotorLog
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
message_duration = []
positions = []
velocities = []
torques = []
cycle_duration = []
send_duration = []
reply_duration = []
child_cycle_duration = []

count = 0
for event in log:
    if event.channel == "motor_data":
        if count > 20:
            msg = ManyMotorLog.decode(event.data)
            timestamps.append(msg.timestamp)
            velocities.append(msg.velocities)
            positions.append(msg.positions)
            margins.append(msg.margin)
            torques.append(msg.torques)
            message_duration.append(msg.message_duration)
        count += 1


timestamps = np.array(timestamps)/1000
margins = np.array(margins)/1000
message_duration = np.array(message_duration)/1000
velocities = np.array(velocities)
positions = np.array(positions)

mean_dt = np.average(np.diff(timestamps))
mean_margin = np.average(margins)
print("Mean dt = ",mean_dt)
print("stdev dt = ",np.std(np.diff(timestamps)))

print("Mean margin = ", mean_margin)
print("std margin = ", np.std(margins))

print("Mean message_duration = ", np.average(message_duration))
print("std message_duration = ", np.std(message_duration))

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
ax1.set_title(file_name)
ax1.plot(timestamps, message_duration)
ax1.set_ylabel('Communication duration')

ax2.plot(timestamps[:-1], np.diff(timestamps))
ax2.set_ylabel('dt(ms)')

ax3.plot(timestamps, torques)
ax3.set_ylabel('Torque(Nm)')

ax4.plot(timestamps, positions)
ax4.set_ylabel('Positions(rad)')

plt.show()


