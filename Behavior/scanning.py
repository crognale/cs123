'''
/* =======================================================================
   scanning.py

   - write servo position
   - read psd distance (calibrated to distance from robot)
   - based on calibration tests, determine angles of detection that pick up
     each box without picking up adjacent boxes
   - construct landmark_id string and output to console

   - can do this more robustly by:
     - store (x,y) coordinates based on a calibration data and the angle,
     that have a proximity value to the robot smaller than a threshold value.
     For coordinates corresponding to proximity values smaller than the threshold
     value, I would perform a count of all such coordinates in each
     area of the map where there could be a box. For any area that has a minimum
     number of coordinates (e.g., 15), I would perform a least square fit on all 
     coordinates in that area. The position and angle of the fit would be used
     to determine the landmarks present, and the position of the robot.

   ========================================================================*/
'''


import behavior
import time
import math

class Behavior(behavior.RobotBehavior):
    def __init__(self, name, robotList, sweep_period, queue):
        super(Behavior, self).__init__(name, robotList)
        self._period = sweep_period
        self._queue = queue

    def write_servo_position(self, robot, deg):

        # write servo position to portB
        portB = deg
        robot.set_io_mode( 1, 0x08 ) # use Analog Servo Control
        robot.set_port(1, portB)

        pass #move the servo motor to deg

    def read_psd_distance(self, robot):

        # read psd distance to portA
        robot.set_io_mode(0, 0x00) # use Analog to Digital Converter mode because we want sensor value that's continuous / analog with range of values
        portA = robot.get_port(0) # this is the reading

        # calibration
        # distance from robot (mm)      portA
        # 150       104
        # 140       109
        # 120       128
        # 100       151
        # 80        178
        # 60        193
        # 40        190
        # 20        98
        # 0         

        return portA #read psd distance 

    def behavior_loop(self):
        print self._name, "behavior_loop() starts!"
        dir = 1
        delta = 1
        deg = 0
        line_psd = {} # stores the lengths of the line
        for i in range(0, 181):
            line_psd[i] = 0
        period = self._period/len(line_psd)*delta
        print "scanning period: 180deg:",period, "sec"
        while (not self._bQuit):
            for robot in self._robotList:
                #print self._name, "behavior_loop(): time = ", time.time(), "sec"

                #move to the desired position: {deg| 0 < deg <=180}
                deg = deg + dir * delta
                offset = 15 # on the sweep back, this gets subtracted from 180
                if deg < 0:
                    deg = 0
                    dir = 1
                elif deg > 180 - offset:
                    deg = 180 - offset
                    dir = -1
                self.write_servo_position(robot, deg) #implement

                #read the psd distance value
                deg2 = deg
                if (dir == -1):
                    deg2 = deg + offset
                rad = deg2 * math.pi / 180.0
                psd_value = self.read_psd_distance(robot) #implement
                mag = 255 - psd_value;

                #draw graphics
                elem = {}
                elem[0] = mag  # length of line
                elem[1] = rad  # angle in radians
                elem[2] = deg2 # angle in degrees
                self._queue.put(elem)
                #print "elem:", elem

                # store magnitude
                line_psd[deg2] = mag

                if deg == 1 and dir == -1:
                    self.calculate_landmarks(line_psd)

                #analyze the sensor data[1 ... 180]

            time.sleep(period)
        print self._name, "behavior_loop() finished!"

    def calculate_landmarks(self, line_psd = []):
        print "calculating landmarks"

        # close: boxes 1,2,5,6
        # far: boxes 3,4
        landmark_thresh_close = 120
        landmark_thresh_far = 150
        landmark_thresh = None

        # landmark_binary stores the landmark_id binary rep
        # landmark_id stores the average mag reading from the sensor for each of 6 ranges
        landmark_binary = 0b000000
        landmark_id = {}

        # define ranges and calculate average magnitude
        # this is based on multiple tests done in a real-scale environment
        for i in range( 0, 6 ):
            landmark_thresh = landmark_thresh_close
            box_start_deg = i * 30
            box_end_deg = (i + 1) * 30
            if i == 0: # box 1
                box_start_deg = 1
                box_end_deg = 15
            if i == 1: # box 2
                box_start_deg = 54
                box_end_deg = 57
            if i == 2: # box 3
                landmark_thresh = 160
            if i == 3: # box 4
                landmark_thresh = landmark_thresh_far
                box_start_deg = 90
                box_end_deg = 100
            if i == 4: # box 5
                landmark_thresh = landmark_thresh_far
                box_start_deg = 118
                box_end_deg = 130
            if i == 5: # box 6
                box_start_deg = 170
                box_end_deg = 176

            line_psd_val = {k: v for k, v in line_psd.iteritems() if box_start_deg < k < box_end_deg}

            landmark_id[i] = sum( line_psd_val.values() ) / ( box_end_deg - box_start_deg )

            landmark_binary = ( landmark_binary << 1 ) + int( landmark_id[i] < landmark_thresh )

            if i==5:
                print box_start_deg, box_end_deg, landmark_thresh, landmark_id, landmark_binary

        print "landmark_id = ", format(landmark_binary, '#08b'), "  raw vals = ", landmark_id


