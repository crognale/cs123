'''
/* =======================================================================
   draw.py

   - draw a race track with two HS robot cars
   - display number of power-ups (boosts) available and their location

   ========================================================================*/
'''

import Queue
import math
import settings
import threading
import scipy.optimize as optimization
import numpy
import time
import eventfsm

monitorevents_timesleep = 0.1

class RobotDraw(object):
    def __init__(self, frame, tk):
        self._frame = frame
        self._points = [36, 40, 600, 400, 20]
        self._canvas = tk.Canvas(frame)
        self._queue = Queue.Queue()
        self._line_l = None
        self._line_r = None
        self._line_psd = {}
        self._line_psd_x = []
        self._line_psd_y = []
        for i in range(0, 181):
            self._line_psd[i] = None

        self.draw_workspace(tk)
        tk.Button(frame, text="Quit", command=quit).pack()
        #tk.Button(frame, text="Line Trace", command=self.linetrace).pack()
        tk.Button(frame, text="Start Race", command=self.startrace).pack()
        tk.Button(frame, text="Stop Threads", command=self.stop).pack()


    def stop(self):
        print "stopping threads..."
        for i in range(0, len(settings.gBehaviors)):
            settings.gBehaviors[i].set_bQuit(True)

    def start(self):
        self.callback_draw()

    # UI initiated events

    def startrace(self):
        print "start button pressed"

        # place event on queue
        event_queue = eventfsm.event_queue
        print event_queue
        if event_queue:
            event = eventfsm.Event("inlane")
            event_queue.put(event)
            time.sleep(monitorevents_timesleep)

    def linetrace(self):
        print "line trace button pressed"
        behavior_threads = []

        # place event on the queue indicating UI press

        # look for behaviors in global list that are collision
        for i in range(0, len(settings.gBehaviors)):
            if (settings.gBehaviors[i].get_name() == "linetrace"):
                settings.gBehaviors[i].set_bQuit(False)
                behavior_threads.append(threading.Thread(target=settings.gBehaviors[i].behavior_loop, name = settings.gBehaviors[i].get_name()))

        for thread in behavior_threads:
            thread.daemon = True
            thread.start()

    def get_queue(self):
        return self._queue

    def draw_workspace(self, tk):
        trackoriginx = 20
        trackoriginy = 20
        trackwidth = 400 * 1.5
        trackheight = 400
        trackcutamount = 100

        pts = self._points
        rect = self._canvas.create_polygon(trackoriginx + trackcutamount, trackoriginy, \
            trackoriginx + trackwidth - trackcutamount, trackoriginy, \
            trackoriginx + trackwidth, trackoriginy + trackcutamount, \
            trackoriginx + trackwidth, trackoriginy + trackheight - trackcutamount, \
            trackoriginx + trackwidth - trackcutamount, trackoriginy + trackheight, \
            trackoriginx + trackcutamount, trackoriginy + trackheight, \
            trackoriginx, trackoriginy + trackheight - trackcutamount, \
            trackoriginx, trackoriginy + trackcutamount, \
            outline="black", fill="white", width=2)

        self._canvas.pack(fill=tk.BOTH, expand=1)

    def callback_draw(self):
        #print "callback_draw()"
        #self.draw_robot()
        self._frame.after(10, self.callback_draw)


