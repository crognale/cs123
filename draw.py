'''
/* =======================================================================
   draw.py

   - create 2 buttons for scanning and collision, respectively
   - press "Global Loc on CCW data" button to output the landmark_id string
     to the console. NOTE: the string reports only data only on the
     counter-clockwise swing, so placement of a box in position '1'
     will not be shown immediately in landmark_id, but will take 1 extra cycle
     press "Collision" to move HS backward and detect collision (work-in-progress)
   - store (x,y) coordinates from the scanning function, to be used with
     least square fit - TODO (used a hard-coded method, refer to scanning.py)
     would use stored coordinates that have a proximity value to the robot
     smaller than a threshold value. For coordinates smaller than the threshold
     value, I would perform a count of all such coordinate in each
     area of the map where there could be a box. For any area that has a minimum
     number of coordinates, I would perform a least square fit on all 
     coordinates in that area. All other areas would be ignored 
   ========================================================================*/
'''

import Queue
import math
import settings
import threading
import scipy.optimize as optimization
import numpy

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
        tk.Button(frame, text="Global Loc on CCW data", command=self.localize).pack()
        tk.Button(frame, text="Collision Moving Bkwds", command=self.collision).pack()
        tk.Button(frame, text="Stop Threads", command=self.stop).pack()

    def stop(self):
        print "stopping threads..."
        for i in range(0, len(settings.gBehaviors)):
            settings.gBehaviors[i].set_bQuit(True)

    def collision(self):
        print "collision button pressed"
        behavior_threads = []

        # look for behaviors in global list that are collision
        for i in range(0, len(settings.gBehaviors)):
            if (settings.gBehaviors[i].get_name() == "collision"):
                settings.gBehaviors[i].set_bQuit(False)
                behavior_threads.append(threading.Thread(target=settings.gBehaviors[i].behavior_loop, name = settings.gBehaviors[i].get_name()))

        for thread in behavior_threads:
            thread.daemon = True
            thread.start()

    def localize(self):
        print "localize button pressed"
        behavior_threads = []

        # look for behaviors in global list that are scanning
        for i in range(0, len(settings.gBehaviors)):
            if (settings.gBehaviors[i].get_name() == "scanning"):
                settings.gBehaviors[i].set_bQuit(False)
                behavior_threads.append(threading.Thread(target=settings.gBehaviors[i].behavior_loop, name = settings.gBehaviors[i].get_name()))

        for thread in behavior_threads:
            thread.daemon = True
            thread.start()

    def start(self):
        self.callback_draw()

    def get_queue(self):
        return self._queue

    def draw_workspace(self, tk):
        pts = self._points
        rect = self._canvas.create_rectangle((pts[2]-pts[0])/2, pts[3]-pts[1], (pts[2]+pts[0])/2, pts[3], outline="red", fill="green", width=2)
        arc_outter = self._canvas.create_arc(pts[2]/2-255-pts[4], pts[3]-pts[1]-255, pts[2]/2+255+pts[4], pts[3]-pts[1]+2*pts[4]+255, start=0, extent=180, outline='red', width=1)
        arc_inner = self._canvas.create_arc((pts[2])/2-pts[4], pts[3]-pts[1], (pts[2])/2+pts[4], pts[3]-pts[1]+2*pts[4], start=0, extent=180, outline='red', width=1)
        self._canvas.pack(fill=tk.BOTH, expand=1)

    def draw_sensors(self):
        pts = self._points
        while not self._queue.empty():
            info = self._queue.get(0)
            if (len(info) == 2):
                if self._line_l != None:
                    self._canvas.delete(self._line_l)
                if self._line_r != None:
                    self._canvas.delete(self._line_r)
                self._line_l = self._canvas.create_line((pts[2]-pts[0])/2, pts[3]-pts[1], (pts[2]-pts[0])/2, pts[3]-pts[1]-info[0], fill="blue", width=2)
                self._line_r = self._canvas.create_line((pts[2]+pts[0])/2, pts[3]-pts[1], (pts[2]+pts[0])/2, pts[3]-pts[1]-info[1], fill="blue", width=2)    
            elif (len(info) == 3):
                mag = info[0] + pts[4]
                rad = math.pi - info[1]
                deg = info[2]
                x = (pts[2])/2 - mag * math.cos(rad)
                y = pts[3]-pts[1]+pts[4] - mag * math.sin(rad)
                x0 = pts[2]/2 - pts[4] * math.cos(rad)
                y0 = pts[3]-pts[1]+pts[4] - pts[4] * math.sin(rad)
                if self._line_psd[deg] != None:
                    self._canvas.delete(self._line_psd[deg])
                    self._line_psd[deg] = None
                if deg % 30 == 0:
                    # mark 30 deg lines in blue for readability
                    self._line_psd[deg] = self._canvas.create_line(x0, y0, x, y, fill="blue", width=1)
                else:
                    self._line_psd[deg] = self._canvas.create_line(x0, y0, x, y, width=1)

                # store the (x,y) coords
                self._line_psd_x.insert( deg, x )
                self._line_psd_y.insert( deg, y )

        #print "getline:", self.get_line(self._line_psd_x, self._line_psd_y)

    def get_line(self, x, y):
        n = len(x)
        m, b = self.least_square_fit(x, y, 0, n)
        print m, b

    def line_func(self, x, A, B):
        return A*x + B

    def least_square_fit(self, x, y, i, f):
        xdata = numpy.array(x[i:f])
        ydata = numpy.array(y[i:f])
        popt, pcov = optimization.curve_fit( self.line_func, xdata, ydata)
        return popt

    def callback_draw(self):
        #print "callback_draw()"
        self.draw_sensors()
        self._frame.after(10, self.callback_draw)


