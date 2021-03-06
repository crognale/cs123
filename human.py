from threading import Thread
import threading
import time
from HamsterAPI.comm_ble import RobotComm
import Tkinter as tk
import Queue
from random import randint
from tk_hamster_GUI import *
import inspect #for debugging
global joystick

import draw
from Behavior import scanning
import settings

BAR_WIDTH = 40
MAX_BAR_SIZE = 150.0

BORDER_WIDTH = 5

MAX_PROXIMITY = 100.0
FOUND_OBJ_THRESHOLD = 40
CLEARED_OBJ_THRESHOLD = 60
ALIGN_SKEW_THRESHOLD = 2

WHEEL_SPEED = 50
TURN_SPEED = 10

FLOOR_BLACK = 60
FLOOR_WHITE = 80

gMaxRobotNum = 1
gRobotList = None

gQuit = False
gKillBehavior = False

gCanvas = None
gCanvas2 = None
frame = None

gProximityBarLeft = None
gProximityBarRight = None
gHamsterBox = None 

gEventQueue = Queue.Queue()
gBeepQueue = Queue.Queue()
#g___Queue[0] is human, [1] is AI
gWheelQueue = [Queue.Queue(), Queue.Queue()]
gProxQueue = Queue.Queue()

FSM = None

UPDATE_INTERVAL = 30
FLAG_LINETRACE = -1

vWorld = None
gTimeOfLastBoost = None
gBoostButtonPressed = False
gCollideButtonPressed = False
gSwitchInnerButtonPressed = False
gSwitchOuterButtonPressed = False
BOOST_DURATION = 2
BOOST_REFRACTORY = 5
COLLIDE_DURATION = 1
joystick = []

gBehaviors = None
PROX_THRESHOLD_DONE = 100 # > 100 = object
FLOOR_THRESHOLD_LANE = 40 # < 40 = black

class State:
    def __init__(self):
      self.name = ''
      self.transitions = []

    def add_transition(self,toState, condition, callback):
      self.transitions.append([toState, condition, callback])

class StateMachine:
    def __init__(self):
        self.states = {}
        self.startState = None
        self.currentState = None

    def add_state(self, name):
        a_state = State()
        a_state.name = name
        self.states[name] = a_state
        return a_state

    def set_start(self, name):
        self.startState = name

    def set_current(self, name):
        self.currentState = name

    # dispatch function
    # executes a callback function that is placed on gEventQueue.
    # callback functions can be put on the queue in the monitor function
    # or the GUI.
    def dispatch(self):
      global gQuit
      global gEventQueue
      global gKillBehavior
      behavior_thread = None

      while not gQuit:
        event = gEventQueue.get(True)
        toState = event[0]
        callback = event[1]
        #Stop current behavior
        gKillBehavior = True
        if (behavior_thread):
          behavior_thread.join()
        gKillBehavior = False
        if (callback != False):
          #Start new behavior
          behavior_thread = threading.Thread(target=callback)
          behavior_thread.daemon = True
          behavior_thread.start()
        self.currentState = toState

    # monitor function
    # places a callback function on gEventQueue based on the
    # transition of the state machine
    def monitor(self):
      global gQuit
      global gEventQueue
      while(not gQuit):
        state = self.states[self.currentState] 
        print "current state", state.name
        for transition in state.transitions:
          if transition[1](): #condition satisfied
            toState = transition[0]
            callback = transition[2]
            gEventQueue.put([toState, callback])
        time.sleep(0.1)


def done():
  global gBeepQueue, gWheelQueue
  finishSong = [76, 71, 64, 71, 76, 71, 64, 71, 76, 0]
  gWheelQueue[0].put([100, -100, 5])
  for i in range(len(finishSong)):
    gBeepQueue.put([finishSong[i], randint(1, 7), randint(1, 7), 0.2])
  gBeepQueue.put([0, 2,2, 0])


def waitForWhite():
  global gBeepQueue, gWheelQueue, gProxQueue
  global gKillBehavior
  global joystick

  # works for 1 robot only
  if joystick[0].vrobot.track == "outer":
    gWheelQueue[0].put([ 2, 3, FLAG_LINETRACE])
  else: # inner
    gWheelQueue[0].put([ 1, 3, FLAG_LINETRACE])

  while (not gKillBehavior):
    gBeepQueue.put([30, 4, 4, 0.1])
    gBeepQueue.put([0, 0, 0, 0.1])

    if joystick[0].vrobot.track == "outer":
      gProxQueue.put([180])
    else: # inner
      gProxQueue.put([1])

    time.sleep(0.2)

def WaitForWhite_to_Boost():
  global gBoostButtonPressed, gTimeOfLastBoost
  if gBoostButtonPressed == True and ( time.time() - gTimeOfLastBoost ) > BOOST_REFRACTORY:
    gBoostButtonPressed = False
    gTimeOfLastBoost = time.time()
    return True
  return False

def boost():
  global gWheelQueue, gBeepQueue
  # 1 sec boost
  gWheelQueue[0].put([ 1, 7, FLAG_LINETRACE])
  boostSongNotes = [47, 52, 56, 59, 56, 59]
  boostSongDuration = [0.5*1/3, 0.5*1/3, 0.5*1/3, 0.5*3/4, 0.5*1/4, 0.5*2]
  for i in range(len(boostSongNotes)):
    gBeepQueue.put([boostSongNotes[i], 2, 2, boostSongDuration[i]])
  gBeepQueue.put([0, 2,2, 0])

  # for i in range(len(boostSongNotes)):
  #   gBeepQueue.put([boostSongNotes[i], 2, 2, boostSongDuration])
  # gBeepQueue.put([0,0,0,0])

def Boost_to_WaitForWhite():
  time.sleep(BOOST_DURATION)
  return True

def WaitForWhite_to_Collide():
  global gCollideButtonPressed, gTimeOfLastBoost
  if gCollideButtonPressed == True:
    gCollideButtonPressed = False
    return True
  return False

def collide():
  global gWheelQueue
  # 1 sec boost
  gWheelQueue[0].put([ 100, 100, 1])

def Collide_to_WaitForWhite():
  time.sleep(COLLIDE_DURATION)
  return True

def WaitForWhite_to_SwitchInner():
  global gSwitchInnerButtonPressed
  if gSwitchInnerButtonPressed == True:
    gSwitchInnerButtonPressed = False
    return True
  return False

def WaitForWhite_to_SwitchOuter():
  global gSwitchOuterButtonPressed
  if gSwitchOuterButtonPressed == True:
    gSwitchOuterButtonPressed = False
    return True
  return False

def SwitchInner_to_WaitForWhite():
  
  for robot in gRobotList:
    if robot.get_floor(0) < FLOOR_THRESHOLD_LANE:
      return True
  return False


def SwitchOuter_to_WaitForWhite():

  for robot in gRobotList:
    if robot.get_floor(0) < FLOOR_THRESHOLD_LANE:
      return True
  return False

def switchInner():
  global gWheelQueue
  global joystick
  gWheelQueue[0].put([ 50, -50, 0.4]) #turn to the right
  gWheelQueue[0].put([50,50,1.2]) #move to other lane

  joystick[0].vrobot.track = "inner"
  print "inner track"


def switchOuter():
  global gWheelQueue
  global joystick
  gWheelQueue[0].put([ -50, 50, 0.4]) #turn to the left
  gWheelQueue[0].put([50,50,1.2]) #move to other lane

  joystick[0].vrobot.track = "outer"

def WaitForWhite_to_Done():
  for robot in gRobotList:

    # read psd distance to portA
    robot.set_io_mode(0, 0x00) # use Analog to Digital Converter mode because we want sensor value that's continuous / analog with range of values
    portA = robot.get_port(0) # read proximity sensor value

    if portA > PROX_THRESHOLD_DONE:
      return True
  return False

def fsm_init():
  global FSM
  FSM = StateMachine()

  State_Start = FSM.add_state("Start")
  State_WaitForWhite = FSM.add_state("WaitForWhite") # line tracing
  State_Boost = FSM.add_state("Boost")
  State_Collide = FSM.add_state("Collide")
  State_SwitchInner = FSM.add_state("SwitchInner")
  State_SwitchOuter = FSM.add_state("SwitchOuter")
  State_Done = FSM.add_state("Done")

  FSM.set_start("Start")
  FSM.set_current("Start")

  State_Start.add_transition("WaitForWhite", lambda: True, waitForWhite)
  State_WaitForWhite.add_transition("Boost", WaitForWhite_to_Boost, boost)
  State_Boost.add_transition("WaitForWhite", Boost_to_WaitForWhite, waitForWhite)
  State_WaitForWhite.add_transition("Collide", WaitForWhite_to_Collide, collide)
  State_Collide.add_transition("WaitForWhite", Collide_to_WaitForWhite, waitForWhite)
  State_WaitForWhite.add_transition("SwitchInner", WaitForWhite_to_SwitchInner, switchInner)
  State_WaitForWhite.add_transition("SwitchOuter", WaitForWhite_to_SwitchOuter, switchOuter)
  State_SwitchInner.add_transition("WaitForWhite", SwitchInner_to_WaitForWhite, waitForWhite)
  State_SwitchOuter.add_transition("WaitForWhite", SwitchOuter_to_WaitForWhite, waitForWhite)
  State_WaitForWhite.add_transition("Done", WaitForWhite_to_Done, done)


def wheel_target(queue_ind):
  global gWheelQueue
  global gQuit
  global joystick

  #Queue element format: [Left wheel, Right wheel, duration/linetrace]
  #If duration == 0, holds indefinitely until next element dequeued
  while not gQuit:
    movement = gWheelQueue[queue_ind].get(True)
    #print 'queue_ind: ', queue_ind, 'movement: ', movement
    #print 'movement: ', movement
    if gRobotList and len(gRobotList) > queue_ind:
      robot = gRobotList[queue_ind]
      robot.set_wheel(0, movement[0])
      robot.set_wheel(1, movement[1])
      robot.set_line_tracer_mode_speed(0,0)

      # joystick[0].vrobot.sl = 30
      # joystick[0].vrobot.sr = 30

      if (movement[2] == FLAG_LINETRACE):
        robot.set_wheel(0, 0)
        robot.set_wheel(1, 0)
        robot.set_line_tracer_mode_speed(movement[0], movement[1])

        if movement[1] == 3:
          joystick[0].vrobot.sl = 21  # 230 mm in 10 s
          joystick[0].vrobot.sr = 21
        elif movement[1] == 7:
          joystick[0].vrobot.sl = 45  # 100 mm in 2.1 s
          joystick[0].vrobot.sr = 45

      elif movement[2] > 0:
        time.sleep(movement[2])
        robot.set_wheel(0, 0)
        robot.set_wheel(1, 0)

def beep_target():
  global gBeepQueue
  global gQuit

  #Queue element format: [note, left LED, right LED, duration]
  #If duration == 0, holds indefinitely until next element dequeued
  while not gQuit:
    beep = gBeepQueue.get(True)
    for robot in gRobotList:
      robot.set_musical_note(beep[0])
      robot.set_led(0, beep[1])
      robot.set_led(1, beep[2])
      if (beep[3] > 0):
        time.sleep(beep[3])
        robot.set_musical_note(0)
        robot.set_led(0, 0)
        robot.set_led(1, 0)

def prox_target():
  global gProxQueue
  global gQuit

  #Queue element format: [degree to turn prox sensor]
  while not gQuit:
    prox = gProxQueue.get(True)
    for robot in gRobotList:

        # write servo position to portB
        portB = prox[0]
        robot.set_io_mode( 1, 0x08 ) # use Analog Servo Control
        robot.set_port(1, portB)

def StartRaceButtonPressed(event=None):
  global monitor_thread, dispatch_thread
  global beep_thread, wheel_threads
  global gNumCleared
  global gTimeOfLastBoost
  gTimeOfLastBoost = time.time() - BOOST_REFRACTORY #allow immediate boost

  if (len(gRobotList) > 0):

    fsm_init()
    monitor_thread = threading.Thread(target=FSM.monitor)
    monitor_thread.daemon = True
    monitor_thread.start()

    dispatch_thread = threading.Thread(target=FSM.dispatch)
    dispatch_thread.daemon = True
    dispatch_thread.start()

    beep_thread = threading.Thread(target = beep_target)
    beep_thread.daemon = True
    beep_thread.start()

    prox_thread = threading.Thread(target = prox_target)
    prox_thread.daemon = True
    prox_thread.start()


    wheel_threads = []
    for i in [0, 1]:
      wheel_threads.append(threading.Thread(target = wheel_target, args=(i,)))
      wheel_threads[i].daemon = True
      wheel_threads[i].start()

def BoostButtonPressed(event=None):
  global gBoostButtonPressed
  global FSM
  if (FSM.currentState == "WaitForWhite"):
    gBoostButtonPressed = True

def CollideButtonPressed(event=None):
  global gCollideButtonPressed
  gCollideButtonPressed = True

def SwitchInnerButtonPressed(event=None):
  global gSwitchInnerButtonPressed
  gSwitchInnerButtonPressed = True

def SwitchOuterButtonPressed(event=None):
  global gSwitchOuterButtonPressed
  gSwitchOuterButtonPressed = True

def draw_track():
  global gCanvas, frame
  trackoriginx = 40
  trackoriginy = 40
  trackwidth = 840
  trackheight = 560
  trackcutamount = 560/4

  #outer track
  outertrack = gCanvas.create_polygon(trackoriginx + trackcutamount, trackoriginy, \
      trackoriginx + trackwidth - trackcutamount, trackoriginy, \
      trackoriginx + trackwidth, trackoriginy + trackcutamount, \
      trackoriginx + trackwidth, trackoriginy + trackheight - trackcutamount, \
      trackoriginx + trackwidth - trackcutamount, trackoriginy + trackheight, \
      trackoriginx + trackcutamount, trackoriginy + trackheight, \
      trackoriginx, trackoriginy + trackheight - trackcutamount, \
      trackoriginx, trackoriginy + trackcutamount, \
      outline="black", fill="white", width=2)

  # inner track
  trackoriginx = 40+80
  trackoriginy = 40+80
  trackwidth = 840 - 80*2
  trackheight = 560 - 80*2
  trackcutamount = (560 - 80*2)/4
  innertrack = gCanvas.create_polygon(trackoriginx + trackcutamount, trackoriginy, \
      trackoriginx + trackwidth - trackcutamount, trackoriginy, \
      trackoriginx + trackwidth, trackoriginy + trackcutamount, \
      trackoriginx + trackwidth, trackoriginy + trackheight - trackcutamount, \
      trackoriginx + trackwidth - trackcutamount, trackoriginy + trackheight, \
      trackoriginx + trackcutamount, trackoriginy + trackheight, \
      trackoriginx, trackoriginy + trackheight - trackcutamount, \
      trackoriginx, trackoriginy + trackcutamount, \
      outline="black", fill="white", width=2)

  trackoriginx = 40
  trackoriginy = 40
  trackwidth = 840
  trackheight = 560
  trackcutamount = 560/4

  line = gCanvas.create_line(trackoriginx, trackheight + 40, trackoriginx + 20, trackheight + 40, width = 3)

  text = gCanvas.create_text(trackoriginx, trackheight + 60, font="Purisa",
            text="20 mm")

  text = gCanvas.create_text(trackoriginx + trackwidth/2, trackheight + 60, font="Purisa",
            text="560 mm")

  text = gCanvas.create_text(trackoriginx + trackwidth + 40, trackoriginy + trackheight/2, font="Purisa",
            text="280 mm")

  gCanvas.pack(fill=tk.BOTH, expand=1)

class VirtualWorldGui:
    def __init__(self, vWorld, m):
        self.vworld = vWorld

        startRaceButton = tk.Button(m,text="Start Race")
        startRaceButton.pack(side='left')
        startRaceButton.bind('<Button-1>', StartRaceButtonPressed)

        boostButton = tk.Button(m,text="Boost >>>")
        boostButton.pack(side='left')
        boostButton.bind('<Button-1>', BoostButtonPressed)

        collideButton = tk.Button(m,text="Collide <!>")
        collideButton.pack(side='left')
        collideButton.bind('<Button-1>', CollideButtonPressed)

        switchInnerButton = tk.Button(m,text="Switch Inner")
        switchInnerButton.pack(side='left')
        switchInnerButton.bind('<Button-1>', SwitchInnerButtonPressed)

        switchOuterButton = tk.Button(m,text="Switch Outer")
        switchOuterButton.pack(side='left')
        switchOuterButton.bind('<Button-1>', SwitchOuterButtonPressed)

        exitButton = tk.Button(m,text="Exit")
        exitButton.pack(side='left')
        exitButton.bind('<Button-1>', stopProg)

    def resetvRobot(self, event=None):
        self.vworld.vrobot.reset_robot()

    def drawMap(self, event=None):
        self.vworld.draw_map()

    def drawGrid(self, event=None):
        x1, y1 = 0, 0
        x2, y2 = self.vworld.canvas_width*2, self.vworld.canvas_height*2
        del_x, del_y = 20, 20
        num_x, num_y = x2 / del_x, y2 / del_y
        # draw center (0,0)
        self.vworld.canvas.create_rectangle(self.vworld.canvas_width-3,self.vworld.canvas_height-3,
                self.vworld.canvas_width+3,self.vworld.canvas_height+3, fill="red")
        # horizontal grid
        for i in range(num_y):
            y = i * del_y
            self.vworld.canvas.create_line(x1, y, x2, y, fill="yellow")
        # verticle grid
        for j in range(num_x):
            x = j * del_x
            self.vworld.canvas.create_line(x, y1, x, y2, fill="yellow")

    def cleanCanvas(self, event=None):
        vcanvas = self.vworld.canvas
        vrobot = self.vworld.vrobot
        vcanvas.delete("all")
        poly_points = [0,0,0,0,0,0,0,0]
        vrobot.poly_id = vcanvas.create_polygon(poly_points, fill='blue')
        vrobot.prox_l_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.prox_r_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.floor_l_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")
        vrobot.floor_r_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")

    def updateCanvas(self, drawQueue):
        self.vworld.canvas.after(UPDATE_INTERVAL, self.updateCanvas, drawQueue)
        while (drawQueue.qsize() > 0):
            drawCommand = drawQueue.get()
            drawCommand()
        
class Joystick:
    def __init__(self, comm, m, gCanvas, vrobot, robot_i=0, keyBindings=['w','s','a','d','x']):
        self.gRobotList = comm.robotList
        self.m = m
        self.vrobot = vrobot
        self.robot_i = robot_i

        self.vrobot.t = time.time()
        
        gCanvas.bind_all('<' + keyBindings[0] + '>', self.move_up)
        gCanvas.bind_all('<' + keyBindings[1] + '>', self.move_down)
        gCanvas.bind_all('<' + keyBindings[2] + '>', self.move_left)
        gCanvas.bind_all('<' + keyBindings[3] + '>', self.move_right)
        gCanvas.bind_all('<' + keyBindings[4] + '>', self.stop_move)  
        gCanvas.pack()

    # joysticking the robot
    def move_up(self, event=None):
        if self.gRobotList and len(self.gRobotList) > self.robot_i:
            robot = self.gRobotList[self.robot_i]
            self.vrobot.sl = 30
            self.vrobot.sr = 30   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_down(self, event=None):
        if self.gRobotList and len(self.gRobotList) > self.robot_i:
            robot = self.gRobotList[self.robot_i]
            self.vrobot.sl = -30
            self.vrobot.sr = -30   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_left(self, event=None):
        if self.gRobotList and len(self.gRobotList) > self.robot_i:
            robot = self.gRobotList[self.robot_i]
            self.vrobot.sl = -15
            self.vrobot.sr = 15   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()       

    def move_right(self, event=None):
        if self.gRobotList and len(self.gRobotList) > self.robot_i:
            robot = self.gRobotList[self.robot_i]
            self.vrobot.sl = 15
            self.vrobot.sr = -15  
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()      

    def stop_move(self, event=None):
        if self.gRobotList and len(self.gRobotList) > self.robot_i:
            robot = self.gRobotList[self.robot_i]
            robot = self.gRobotList[self.robot_i]
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def update_virtual_robot(self):

        #---- model the robot
        #---- tests:
        # Location of test: Gates B21 paper-based grid
        # time = 5 second
        # distance robot moved = 160 mm
        # speed = 160 mm / 5 s = 32 mm / s   (v robot speed = 30 mm / s)
        # d_factor = distance / time / virtual speed = 1.0667
        # d_factor = 32 mm/s / 30 mm/s = 1.0667
        # 

        #---- a_factor_cw
        #---- tests:
        # Location of test: Gates B21 paper-based grid
        # time = 7 seconds
        # angle robot moved = 2*pi radians
        # angular speed = 15 (wheel speed) / (2*pi rad / 7 sec) = 16.71

        #---- a_factor2_ccw
        #---- tests:
        # Location of test: Gates B21 paper-based grid
        # time = 7.8 seconds
        # angle robot moved = 2*pi radians
        # angular speed = 15 (wheel speed) / (2*pi rad / 7.8 sec) = 18.6

        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 #floor ambient color - if floor is darker, set higher noise
        p_factor = 1.6 #proximity conversion - assuming linear #orig = 1.4
        d_factor = 1.0667 #travel distance conversion  -  make bigger if fake robot is slower   # have robot move for 10 seconds. record distance.
        # d_factor = speed of the robot
        a_factor_cw = 16.71 # rotation conversion, assuming linear
        a_factor2_ccw = 16

        while not self.gRobotList:
            print "waiting for robot to connect"
            time.sleep(0.1)

        print "connected to robot"

        while not gQuit:
            if self.gRobotList and len(self.gRobotList) > self.robot_i:
                robot = self.gRobotList[self.robot_i]

                t = time.time()
                del_t = t - self.vrobot.t
                self.vrobot.t = t # update the tick
                if self.vrobot.sl == self.vrobot.sr: #---- speed of left wheel = speed of right wheel
                    #--- update x,y position
                    self.vrobot.x = self.vrobot.x + self.vrobot.sl * del_t * math.sin(self.vrobot.a) * d_factor
                    self.vrobot.y = self.vrobot.y + self.vrobot.sl * del_t * math.cos(self.vrobot.a) * d_factor
                if self.vrobot.sl == -self.vrobot.sr:
                    #--- update angle
                    #--- if sr > 0 and sl < 0, rotates clockwise
                    #--- if sr < 0 and sl > 0, rotates counter-clockwise
                    if (self.vrobot.sl > 0): self.vrobot.a = self.vrobot.a + (self.vrobot.sl * del_t)/a_factor_cw 
                    if (self.vrobot.sl < 0): self.vrobot.a = self.vrobot.a + (self.vrobot.sl * del_t)/a_factor2_ccw

                #---update position if line tracer is being used---#
                
                #---update sensors and get dist_l and dist_r
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
                if (prox_l > noise_prox):
                    self.vrobot.dist_l = (100 - prox_l)*p_factor
                else:
                    self.vrobot.dist_l = False
                if (prox_r > noise_prox):
                    self.vrobot.dist_r = (100 - prox_r)*p_factor
                else:
                    self.vrobot.dist_r = False
                
                floor_l = robot.get_floor(0)
                floor_r = robot.get_floor(1)
                if (floor_l < noise_floor):
                    self.vrobot.floor_l = floor_l
                else:
                    self.vrobot.floor_l = False
                if (floor_r < noise_floor):
                    self.vrobot.floor_r = floor_r
                else:
                    self.vrobot.floor_r = False

            time.sleep(0.1)


def stopProg(event=None):
    global gQuit
    global frame
    frame.quit()
    gQuit = True
    print "Exit"

def draw_virtual_world(virtual_world):
    time.sleep(1) # give time for robot to connect.
    while not gQuit:
        if gRobotList is not None:
            virtual_world.draw_robot()
            virtual_world.draw_prox("left")
            virtual_world.draw_prox("right")
            virtual_world.store_prox()
            virtual_world.draw_floor("left")
            virtual_world.draw_floor("right")

            # draw scanning output in virtual_world
            # get value of scanning from gBehavior variable
            
            # look for behaviors in global list that are scanning
            for i in range(0, len(settings.gBehaviors)):
              if (settings.gBehaviors[i].get_name() == "scanning"):

                # unable to get landmarks from scanning behavior
                landmarks = settings.landmarks
                for landmarks_index in range(6):
                  pi6 = 3.1415 / 6
                  angle_from_robot = landmarks_index * pi6 + pi6/2 #radians
                  if (landmarks >> landmarks_index) & 0b000001 == 1:
                    print "landmark drawn!!"
                    # draw landmark; more robust if implemented as a function of virtual_world
                    virtual_world.draw_text_around_robot("obs", angle_from_robot, 40)

        time.sleep(0.1)

def main():
  global gRobotList, gQuit
  global gCanvas, frame, gHamsterBox
  global monitor_thread, dispatch_thread
  global gBeepQueue, gWheelQueue, drawQueue
  global vWorld
  global joystick

  comm = RobotComm(gMaxRobotNum)
  comm.start()
  print 'Bluetooth started'

  gRobotList = comm.robotList

  monitor_thread = False
  dispatch_thread= False

  # create UI: two separate Tkinter windows
  # 1. frame = course track display
  # 2. gFrame = localization scanning display
  frame = tk.Tk()
  canvas_width = 700 # half width
  canvas_height = 380 # half height
  gCanvas = tk.Canvas(frame, bg="white", width=canvas_width*2, height=canvas_height*2)
  draw_track()

  settings.init()
  settings.gFrame = tk.Tk()
  settings.gFrame.geometry('600x500')
  gRobotDraw = draw.RobotDraw(settings.gFrame, tk)
  # create scanning behavior
  settings.gBehaviors[0] = scanning.Behavior("scanning", gRobotList, 4.0, gRobotDraw.get_queue())

  gRobotDraw.start()

  # create 2 virtual robot data objects
  vrobot = []
  joystick = []
  keyBindings = []
  for robot_i in range(gMaxRobotNum):
    vrobot.append ( virtual_robot() )
    pi4 = 3.1415 / 4

    # robot starting positions
    vrobot[robot_i].set_robot_a_pos(pi4*2, -520 + robot_i * 40, +340 - robot_i * 80)

    # keyboard input
    if robot_i == 0:
      keyBindings = ['w','s','a','d','x']
    elif robot_i == 1:
      keyBindings = ['i','k','j','l',',']
    joystick.append( Joystick(comm, frame, gCanvas, vrobot[robot_i], robot_i, keyBindings) )
    poly_points = [0,0,0,0,0,0,0,0]
    joystick[robot_i].vrobot.poly_id = gCanvas.create_polygon(poly_points, fill='blue') #robot
    joystick[robot_i].vrobot.prox_l_id = gCanvas.create_line(0,0,0,0, fill="red") #prox sensors  ---- here
    joystick[robot_i].vrobot.prox_r_id = gCanvas.create_line(0,0,0,0, fill="red")
    joystick[robot_i].vrobot.floor_l_id = gCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
    joystick[robot_i].vrobot.floor_r_id = gCanvas.create_oval(0,0,0,0, outline="white", fill="white")

    time.sleep(1)

    update_vrobot_thread = threading.Thread(target=joystick[robot_i].update_virtual_robot)
    update_vrobot_thread.daemon = True
    update_vrobot_thread.start()

  # virtual world UI
  drawQueue = Queue.Queue(0)
  vWorld = virtual_world(drawQueue, joystick[0], vrobot[0], gCanvas, canvas_width, canvas_height)
  landmark = [-500, 220, -460, 180]
  vWorld.add_obstacle(landmark)

  draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld,))
  draw_world_thread.daemon = True
  draw_world_thread.start()

  gui = VirtualWorldGui(vWorld, frame)

  gui.drawGrid()
  gui.drawMap()

  gCanvas.after(200, gui.updateCanvas, drawQueue)
  frame.mainloop()

  gQuit = True

  for robot in gRobotList:
    robot.reset()
  time.sleep(1.0)
  comm.stop()
  comm.join()

  print 'Terminated'

if __name__ == "__main__":
    main()
