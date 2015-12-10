'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''


'''
/* =======================================================================
   Assignment 4

   - Jamy Li 05442431
   - Implementation Notes:
   - Part 1: Scanning
      - Refer to scanning.py for write_servo_position and read_psd_distance functions
      - Calibration tests for the sensor's movement-writing and distance-reading are placed as comments in those functions
   - Part 2: Collision
      - Refer to collision.py
   - UI/UX: Scanning and Collision detection are performed using buttons implemented with the Tk GUI
      - Refer to draw.py
   ========================================================================*/
'''
# 


import sys
import signal
import threading
import time
import Tkinter as tk
import settings

from HamsterAPI.comm_ble import RobotComm
#from HamsterAPI.comm_usb import RobotComm
import draw
from Behavior import motion, color, sound, proxy, scanning, collision

def quit():
    print "quitting..."
    for i in range(0, len(settings.gBehaviors)):
        settings.gBehaviors[i].set_bQuit(True)
    time.sleep(1)
    settings.gFrame.quit()

def clean_up():
    quit()
    print "cleaning up..."

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    clean_up()

signal.signal(signal.SIGINT, signal_handler)

def main(argv=None):
    # instantiate COMM object
    comm = RobotComm(1, -50) #maxRobot = 1, minRSSI = -50
    if comm.start():
        print 'Communication starts'
    else:
        print 'Error: communication'
        return

    # instantiate Robot
    robotList = comm.get_robotList()

    # instantiate global variables gFrame and gBehavior
    settings.init()

    settings.gFrame = tk.Tk()
    settings.gFrame.geometry('600x500')

    gRobotDraw = draw.RobotDraw(settings.gFrame, tk)

    # create behaviors
    settings.gBehaviors[0] = scanning.Behavior("scanning", robotList, 4.0, gRobotDraw.get_queue())
    settings.gBehaviors[1] = collision.Behavior("collision", robotList, -50)

    gRobotDraw.start()
    settings.gFrame.mainloop()
    
    for behavior in behavior_threads:
        print "joining... ", behavior.getName()
        behavior.join()
        print behavior.getName(), "joined!"

    for robot in robotList:
        robot.reset()

    comm.stop()
    comm.join()

    print("terminated!")

if __name__ == "__main__":
    sys.exit(main())
