'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   PROPRIETARY and CONFIDENTIAL
   ========================================================================*/
'''

import sys
import signal
import threading
import time
import Tkinter as tk
import settings
import Queue
import draw
from eventfsm import *

from HamsterAPI.comm_ble import RobotComm
#from HamsterAPI.comm_usb import RobotComm
from draw import *
from Behavior import proxy, linetrace

quit = False
event_queue = None
comm = None
behavior_threads = []

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

    # SETUP ROBOT
    comm = RobotComm(1, -50) #maxRobot = 2, minRSSI = -50
    if comm.start():
        print 'Communication starts'
    else:
        print 'Error: communication'
        return
    robotList = comm.get_robotList()

    # SETUP EVENT-DRIVEN FSM
    global event_queue
    event_queue = Queue.Queue()
    print event_queue

    fsm = EventFsm()
    print "created a state machine"
    build_states(fsm)

    fsm.set_start("start")
    fsm.set_current("start")

    event_thread = threading.Thread(target=monitor_events) #in fsm
    event_thread.start()
    print "monitor_events thread started"

    fsm_thread = threading.Thread(target=fsm.run) #in fsm
    fsm_thread.start()
    print "fsm handler thread started"

    # SETUP CANVAS AND BEHAVIOR LIST
    settings.init()
    settings.gFrame = tk.Tk()
    settings.gFrame.geometry('1200x1000')
    gRobotDraw = draw.RobotDraw(settings.gFrame, tk)
    settings.gBehaviors[0] = linetrace.Behavior("linetrace", robotList, -50)

    gRobotDraw.start()
    settings.gFrame.mainloop()
    
    # JOINING THREADS UPON QUIT
    event_thread.join()

    for behavior in gBehaviors:
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
