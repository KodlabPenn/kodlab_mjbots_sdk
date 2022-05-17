import lcm
from lcm_types.ManyMotorLog import ManyMotorLog
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(suppress=True)
from matplotlib.animation import FuncAnimation
import collections
import time

timestamps = collections.deque(np.zeros(3000))
margins = collections.deque(np.zeros(3000))
message_duration = collections.deque(np.zeros(3000))
positions = collections.deque(np.zeros((3000,13)))
velocities = collections.deque(np.zeros((3000,13)))
torques = collections.deque(np.zeros((3000,13)))


def update_plot():

    ax1.cla()
    ax2.cla()
    ax3.cla()
    ax4.cla()
    ax5.cla()

    ax1.set_title("Twist Leg Configuration")
    ax1.set_ylabel("Leg 0")
    ax2.set_ylabel("Leg 1")
    ax3.set_ylabel("Leg 2")
    ax4.set_ylabel("Leg 3")
    ax5.set_ylabel("Spine")

    time = np.array(timestamps)/1e6
    print(time[-1])
    pos = np.array(positions)
    vel = np.array(velocities)
    mar = np.array(margins)
    tor = np.array(torques)
    dur = np.array(message_duration)

    ax1.plot(time, pos[:,0:3])
    ax2.plot(time, pos[:,3:6])
    ax3.plot(time, pos[:,6:9])
    ax4.plot(time, pos[:,9:12])
    ax5.plot(time, pos[:,12])

    plt.draw()
    plt.pause(0.0001)


def my_handler(channel, data):

    msg = ManyMotorLog.decode(data)
    timestamps.popleft()
    velocities.popleft()
    positions.popleft()
    margins.popleft()
    torques.popleft()
    message_duration.popleft()
    msg
    timestamps.append(msg.timestamp)
    velocities.append(msg.velocities)
    positions.append(msg.positions)
    margins.append(msg.margin)
    torques.append(msg.torques)
    message_duration.append(msg.message_duration)


if __name__ == '__main__':

    lc = lcm.LCM()

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1)

    plt.show(block=False)

    lastTime=0
    subscription = lc.subscribe("motor_data", my_handler)

    try:
        count = 0
        lc.handle()
        while True:
            if count>20:
                lc.handle_timeout(500)
            count += 1
            currTime = int(time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)/1e6) #millis
            if currTime - lastTime > 500:
                update_plot()
                lastTime = currTime
            

    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)