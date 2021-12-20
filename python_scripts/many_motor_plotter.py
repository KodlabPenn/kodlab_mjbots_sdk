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
            cycle_duration.append(msg.cycle_duration)
            send_duration.append(msg.send_duration)
            reply_duration.append(msg.reply_duration)
            child_cycle_duration.append(msg.child_cycle_duration)
        count += 1


timestamps = np.array(timestamps)/1000
margins = np.array(margins)/1000
message_duration = np.array(message_duration)/1000
velocities = np.array(velocities)
positions = np.array(positions)

reply_duration = np.array(reply_duration)/1000
send_duration = np.array(send_duration)/1000
cycle_duration = np.array(cycle_duration)/1000
child_cycle_duration = np.array(child_cycle_duration)/1000

mean_dt = np.average(np.diff(timestamps))
mean_margin = np.average(margins)
print("Mean dt = ",mean_dt)
print("stdev dt = ",np.std(np.diff(timestamps)))

print("Mean margin = ", mean_margin)
print("std margin = ", np.std(margins))

print("Mean message_duration = ", np.average(message_duration))
print("std message_duration = ", np.std(message_duration))

print("Mean child cycle duration = ", np.average(child_cycle_duration))
print("Mean pi3 hat cycle duration = ", np.average(cycle_duration))
print("Mean send_duration = ", np.average(send_duration))
print("Mean reply_duration = ", np.average(reply_duration))

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
ax1.set_title(file_name)
ax1.plot(timestamps, message_duration)
ax1.set_ylabel('Communication duration')

ax2.plot(timestamps[:-1], np.diff(timestamps))
ax2.set_ylabel('dt(ms)')

ax3.plot(timestamps, cycle_duration)
ax3.set_ylabel('duration(s)')
ax3.plot(timestamps, send_duration)
ax3.plot(timestamps, reply_duration)
ax3.plot(timestamps, child_cycle_duration)

ax4.plot(timestamps, positions)
ax4.set_ylabel('Positions(rad)')

plt.show()


