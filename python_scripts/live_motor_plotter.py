import lcm
from lcm_types.ManyMotorLog import ManyMotorLog
import matplotlib.pyplot as plt
import numpy as np
import collections
import time
from blit_manager import BlitManager

# NumPy Options
np.set_printoptions(suppress=True)

# Constants
PLIM = np.pi
VLIM = 10
TLIM = 0.01
WINDOWLENGTH = 2000
PLOTPERIOD = 1000
TIMEOUT = 500

# Queues
timestamps = collections.deque(np.zeros(WINDOWLENGTH))
margins = collections.deque(np.zeros(WINDOWLENGTH))
message_duration = collections.deque(np.zeros(WINDOWLENGTH))
positions = collections.deque(np.zeros((WINDOWLENGTH,13)))
velocities = collections.deque(np.zeros((WINDOWLENGTH,13)))
torques = collections.deque(np.zeros((WINDOWLENGTH,13)))
velo_filt = collections.deque(np.zeros((WINDOWLENGTH,13)))


def init_plot():

    fig, axs = plt.subplots(6, 2, sharex=True) # Plot two columns
    axs = np.concatenate((axs[:,0] ,axs[:,1] )) # Make single vector

    axs[0].set_title("Positions")
    for i in range(4):
        axs[i].set_ylabel(f'Leg {i}')
        axs[i].set_ylim(-PLIM, PLIM)
    axs[4].set_ylabel("Spine")
    axs[4].set_ylim(-PLIM, PLIM)
    axs[5].set_ylabel("dt")
    axs[5].set_ylim(0, TLIM)

    axs[6].set_title("Velocities")
    for i in range(4):
        axs[6 + i].set_ylim(-VLIM, VLIM) # Leg joint velocities limits
    axs[10].set_ylim(-VLIM, VLIM)

    times = np.array(timestamps)/1e6
    pos = np.array(positions)
    vel = np.array(velocities)
    mar = np.array(margins)
    tor = np.array(torques)
    dur = np.array(message_duration)
    # Initialize line list
    lns=[]
    # Get leg position lines
    for i in range(4):
        lns.extend( axs[i].plot(times, pos[:, 3*i:3*(i+1)], animated=True) )
    # Get spine positions line
    lns.extend( axs[4].plot(times, pos[:,12], animated=True))
    # Get dts
    lns.extend( axs[5].plot(times,np.concatenate(([0],np.diff(times))), animated=True))
    # Get leg velocities lines
    for i in range(4):
        lns.extend( axs[6 + i].plot(times, vel[:, 3*i:3*(i+1)], animated=True))
    # Get spine velocity line
    lns.extend( axs[10].plot(times, vel[:,12], animated=True))

    return fig,axs,lns

def update_plot_lines(lns):

    times = np.array(timestamps)/1e6
    pos = np.array(positions)
    vel = np.array(velocities)
    mar = np.array(margins)
    tor = np.array(torques)
    dur = np.array(message_duration)

    for ln in lns:
        ln.set_xdata(times)

    for i in range(13):
        lns[i].set_ydata(pos[:, i])

    i += 1
    lns[i].set_ydata(np.concatenate(([0],np.diff(times))))

    i += 1
    for i, j in zip(range(i, i + 12), range(12)):
        lns[i].set_ydata(vel[:, j])

    min_x = np.min(times)
    max_x = np.max(times)
    for ax in axs:
        ax.set_xlim(min_x,max_x)


def my_handler(channel, data):

    msg = ManyMotorLog.decode(data)
    timestamps.popleft()
    velocities.popleft()
    positions.popleft()
    margins.popleft()
    torques.popleft()
    message_duration.popleft()

    timestamps.append(msg.timestamp)
    velocities.append(msg.velocities)
    positions.append(msg.positions)
    margins.append(msg.margin)
    torques.append(msg.torques)
    message_duration.append(msg.message_duration)


if __name__ == '__main__':

    lc = lcm.LCM()
    subscription = lc.subscribe("motor_data", my_handler)
    fig, axs, lns = init_plot() #lns is list of lines
    bm = BlitManager(fig.canvas,lns)

    plt.show(block=False)
    plt.pause(.1)

    lastTime=0

    try:
        count = 0
        lc.handle()
        while True:
            if count>20: #ignore first few signals
                lc.handle_timeout(TIMEOUT)
            count += 1
            currTime = int(time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)/1e6) #millis
            if currTime - lastTime > PLOTPERIOD:

                update_plot_lines(lns)

                bm.update()

                plt.draw()
                plt.pause(.0001)
                lastTime = currTime


    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)