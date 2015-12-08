'''
/* =======================================================================
   draw.py

   - draw a race track with two HS robot cars
   - display number of power-ups (boosts) available and their location

   ========================================================================*/
'''

import Queue
import math
import threading
import scipy.optimize as optimization
import numpy
import time
from proj import gEventQueue, StartClean, done

monitorevents_timesleep = 0.1

class RobotDraw(object):
    def __init__(self, frame, tk):
        self._frame = frame
        self._points = [36, 40, 600, 400, 20]
        self._canvas = tk.Canvas(frame)
        self._line_l = None
        self._line_r = None
        self._line_psd = {}
        self._line_psd_x = []
        self._line_psd_y = []
        for i in range(0, 181):
            self._line_psd[i] = None

        self.draw_workspace(tk)
        tk.Button(frame, text="Quit", command=quit).pack()
        tk.Button(frame, text="Start Race", command=self.startrace).pack()
        tk.Button(frame, text="Stop Threads", command=self.stop).pack()

    def start(self):
        self.callback_draw()

    # UI initiated events

    def startrace(self):
        gEventQueue.put( "start", StartClean )
        print "start button pressed"

    def linetrace(self):
        print "line trace button pressed"

    def stop(self):
        print "stop threads button pressed"
        gEventQueue.put( "done", done )

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


