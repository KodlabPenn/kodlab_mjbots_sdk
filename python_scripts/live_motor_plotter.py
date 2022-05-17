from xml.etree.ElementTree import PI
import lcm
from lcm_types.ManyMotorLog import ManyMotorLog
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(suppress=True)
from matplotlib.animation import FuncAnimation
import collections
import time

timestamps = collections.deque(np.zeros(1000))
margins = collections.deque(np.zeros(1000))
message_duration = collections.deque(np.zeros(1000))
positions = collections.deque(np.zeros((1000,13)))
velocities = collections.deque(np.zeros((1000,13)))
torques = collections.deque(np.zeros((1000,13)))

class BlitManager:
    def __init__(self, canvas, animated_artists=()):
        """
        Parameters
        ----------
        canvas : FigureCanvasAgg
            The canvas to work with, this only works for sub-classes of the Agg
            canvas which have the `~FigureCanvasAgg.copy_from_bbox` and
            `~FigureCanvasAgg.restore_region` methods.

        animated_artists : Iterable[Artist]
            List of the artists to manage
        """

        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def on_draw(self, event):
        """Callback to register with 'draw_event'."""
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        """
        Add an artist to be managed.

        Parameters
        ----------
        art : Artist

            The artist to be added.  Will be set to 'animated' (just
            to be safe).  *art* must be in the figure associated with
            the canvas this class is managing.

        """
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        """Draw all of the animated artists."""
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        """Update the screen with animated artists."""
        cv = self.canvas
        fig = cv.figure
        # paranoia in case we missed the draw event,
        if self._bg is None:
            self.on_draw(None)
        else:
            # restore the background
            cv.restore_region(self._bg)
            # draw all of the animated artists
            self._draw_animated()
            # update the GUI state
            cv.blit(fig.bbox)
        # let the GUI event loop process anything it has to do
        cv.flush_events()

    

def init_plot():

    fig, axs = plt.subplots(6, 1)

    axs[0].set_title("Twist Leg Configuration")
    axs[0].set_ylabel("Leg 0")
    axs[0].set_ylim(-np.pi,np.pi)
    axs[1].set_ylabel("Leg 1")
    axs[1].set_ylim(-np.pi,np.pi)
    axs[2].set_ylabel("Leg 2")
    axs[2].set_ylim(-np.pi,np.pi)
    axs[3].set_ylabel("Leg 3")
    axs[3].set_ylim(-np.pi,np.pi)
    axs[4].set_ylabel("Spine")
    axs[4].set_ylim(-np.pi,np.pi)
    axs[5].set_ylabel("dt")
    axs[5].set_ylim(0,.01)

    times = np.array(timestamps)/1e6
    pos = np.array(positions)
    vel = np.array(velocities)
    mar = np.array(margins)
    tor = np.array(torques)
    dur = np.array(message_duration)
    lns=[]
    lns.extend( axs[0].plot(times, pos[:,0:3], animated=True))
    lns.extend( axs[1].plot(times, pos[:,3:6], animated=True))
    lns.extend( axs[2].plot(times, pos[:,6:9], animated=True))
    lns.extend( axs[3].plot(times, pos[:,9:12], animated=True))
    lns.extend( axs[4].plot(times, pos[:,12], animated=True))
    lns.extend( axs[5].plot(times,np.concatenate(([0],np.diff(times))), animated=True))

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
    
    lns[0].set_ydata(pos[:,0])
    lns[1].set_ydata(pos[:,1])
    lns[2].set_ydata(pos[:,2])
    lns[3].set_ydata(pos[:,3])
    lns[4].set_ydata(pos[:,4])
    lns[5].set_ydata(pos[:,5])
    lns[6].set_ydata(pos[:,6])
    lns[7].set_ydata(pos[:,7])
    lns[8].set_ydata(pos[:,8])
    lns[9].set_ydata(pos[:,9])
    lns[10].set_ydata(pos[:,10])
    lns[11].set_ydata(pos[:,11])
    lns[12].set_ydata(pos[:,12])
    lns[13].set_ydata(np.concatenate(([0],np.diff(times))))
    
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
            if count>20:
                lc.handle_timeout(500)
            count += 1
            currTime = int(time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)/1e6) #millis
            if currTime - lastTime > 500:
                
                update_plot_lines(lns)

                bm.update()

                plt.draw()
                plt.pause(.0001)
                lastTime = currTime
            

    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)