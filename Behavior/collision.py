'''
/* =======================================================================
   collision.py

   - get accelerometer data from the Hamster
   - apply a low pass filter with a low cutoff frequency to eliminate noise
   - detect collision with a solid object by checking whether the low pass filter
   output is close to zero (i.e., robot has stopped ac)


   - based on testing, it looks like the robot's accelerometer axis is:

                      ^  + x accel
                      |
                    _.._
     + y accel  <-  |HS|  ->  - y accel
                    ----
                      |
                      v  - x accel
                      Force

    - if a collision occurs, the force will be in the + x direction, so we expect
      get_acceleration(0) to be large if its a head-on collision
    - any collision should have a +ve x accel component

   ========================================================================*/
'''

import behavior
import time
import math

class Behavior(behavior.RobotBehavior):
    def __init__(self, name, robotList, default_speed = -50):
        super(Behavior, self).__init__(name, robotList)
        self._default_speed = default_speed 
        self._accelx_list = []
        self._accely_list = []
        self._accel_mag = []

    def behavior_loop(self):
        print self._name, "behavior_loop() starts!"

        # threshold for detecting force from impact
        accel_thresh = 30 # optional
        accelx_thresh = 500

        # low pass filter characteristics
        pi = 3.1415
        fc = 1.5 #Hz
        RC = 1 / ( fc * 2 * pi )

        # dt assumed to be time sleep
        tsleep = .01 #0.0001

        # calculate magnitude of collision using accelerometer data
        speed = self.get_speed()
        while (not self._bQuit):
            for robot in self._robotList:
                print self._name, "behavior_loop(): time = ", time.time(), "sec"

                # move backward
                robot.set_wheel(0, speed)
                robot.set_wheel(1, speed)

                # get accelerometer data (instantaneous)
                accelx = robot.get_acceleration(0)
                accely = robot.get_acceleration(1)
                accelz = robot.get_acceleration(2)

                # append to a historical list
                self._accelx_list.append( accelx )
                self._accely_list.append( accely )
                mag = math.sqrt( accelx ** 2 + accely ** 2 )
                self._accel_mag.append( mag )

                # apply low pass filters to accelerometer data
                # to remove noise
                lpx= self.low_pass( self._accelx_list, tsleep, RC )
                lpy= self.low_pass( self._accely_list, tsleep, RC )
                lpoutput = self.low_pass( self._accel_mag, tsleep, RC )

                # test whether an object is pushing back on HS
                isForceBackward = lpx[len(lpx) - 1] > accelx_thresh # the force should be in the +ve x direction if encountering object

                # debugging
                #print "accel x y z", accelx, accely, accelz
                #print "                                         lpout x y m", lpx[len(lpx)-1], lpy[len(lpy)-1], lpoutput[len(lpoutput) - 1]

                # check if the last magnitude as outputted from the low pass filter is large and in the opposite direction of movement
                if lpoutput[len(lpoutput) - 1] > accel_thresh and isForceBackward:
                    print "OBSTACLE detected with magnitude and angle..."
                    print lpoutput[len(lpoutput) - 1]
                    print math.atan2( accely, accelx )
                    print "accel x lp output:", lpx[len(lpx) - 1]
                    self._bQuit = True



                # low pass filter both x and y accel. If ( current low pass filter value - last low pass filter value ) exceeds a certain threshold,
                # a collision is detected. This means that something has happened where the acceleration of the robot has changed in the x-y plane.
                # this is in contrast to my method, which detects whether the absolute value of the acceleration is above a value.



                #time.sleep(1.0)

            time.sleep(tsleep)

        # tasks to be executed when thread closes
        for robot in self._robotList:
            robot.set_wheel(0,0)
            robot.set_wheel(1,0)
            self._accelx_list = []
            self._accely_list = []
            self._accel_mag = []


        print self._name, "behavior_loop() finished!"

    def get_speed(self):
        return self._default_speed

    def low_pass(self, x, dt, RC ):
        # output samples y, input samples x, time inteval dt, time constant RC
        y = []
        a = dt / (RC + dt)
        y.append( x[0] )

        for i in range(1, len(x)):
            y.append ( a * x[i] + ( 1 - a ) * y[i-1] )

        return y



