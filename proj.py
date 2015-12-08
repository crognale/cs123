from threading import Thread
import threading
import time
from HamsterAPI.comm_ble import RobotComm
import Tkinter as tk
import Queue
from random import randint
from tk_hamster_GUI import *

CANVAS_SIZE = 300
BOX_SIZE = 100
BOX_X = int((CANVAS_SIZE / 2) - (BOX_SIZE / 2))
BOX_Y = int((CANVAS_SIZE / 2) + (BOX_SIZE / 2))

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

CLEAR_TARGET = 3
gNumCleared = 0

gQuit = False
gKillBehavior = False

gCanvas = None
frame = None

gProximityBarLeft = None
gProximityBarRight = None
gHamsterBox = None 

gEventQueue = Queue.Queue()
gBeepQueue = Queue.Queue()
gWheelQueue = Queue.Queue()

FSM = None

UPDATE_INTERVAL = 30
FLAG_LINETRACE = -1

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

def updateHamsterBox(left, right):
  global gCanvas, gHamsterBox

  average = (left + right) / 2.0
  shade = int(255.0 * (average / 100.0))
  rgb = "#%02x%02x%02x" % (shade, shade, shade)
  gCanvas.itemconfig(gHamsterBox, fill=rgb)

def updateProximityBars(left, right):
  global gCanvas, gProximityBarLeft, gProximityBarRight
  leftSize = (1 - (left / MAX_PROXIMITY)) * MAX_BAR_SIZE
  rightSize = (1 - (right / MAX_PROXIMITY)) * MAX_BAR_SIZE

  gCanvas.coords(gProximityBarLeft, BOX_X+BAR_WIDTH, BOX_Y,
      BOX_X, BOX_Y - leftSize)
  gCanvas.coords(gProximityBarRight, BOX_X+BOX_SIZE, BOX_Y,
      BOX_X+BOX_SIZE-BAR_WIDTH, BOX_Y - rightSize)

def createProximityBars():
  global gCanvas, gProximityBarLeft, gProximityBarRight
  gProximityBarLeft = gCanvas.create_rectangle(BOX_X+BAR_WIDTH, BOX_Y,
      BOX_X, BOX_Y - MAX_BAR_SIZE, fill="white", width=BORDER_WIDTH)
  gProximityBarRight = gCanvas.create_rectangle(BOX_X+BOX_SIZE, BOX_Y,
      BOX_X+BOX_SIZE-BAR_WIDTH, BOX_Y - MAX_BAR_SIZE, fill="white",
      width=BORDER_WIDTH)


def done():
  global gBeepQueue, gWheelQueue
  finishSong = [76, 71, 64, 71, 76, 71, 64, 71, 76, 0]
  gWheelQueue.put([100, -100, 5])
  for i in range(len(finishSong)):
    gBeepQueue.put([finishSong[i], randint(1, 7), randint(1, 7), 0.2])
  gBeepQueue.put([0, 2,2, 0])

def clearedTarget():
  return gNumCleared >= CLEAR_TARGET

def backUpDelay():
  time.sleep(0.5)
  return True

def backUp():
  global gBeepQueue, gWheelQueue
  global gNumCleared
  gWheelQueue.put([-100, -100, 0.5])

def foundObjectClose():
  for robot in gRobotList:
    left = robot.get_proximity(0)
    right = robot.get_proximity(1)
    return (left > CLEARED_OBJ_THRESHOLD or right > CLEARED_OBJ_THRESHOLD)

def checkSuccess():
  gWheelQueue.put([0, 0, 0])

def turnAroundFull():
  global gBeepQueue, gWheelQueue
  global gNumCleared
  gNumCleared += 1
  for x in range(gNumCleared):
    gBeepQueue.put([70, 2, 2, 0.1])
    gBeepQueue.put([0, 0, 0, 0.1])
    duration = randint(5, 10) / 10.0
  gWheelQueue.put([-WHEEL_SPEED ,WHEEL_SPEED, duration])
  time.sleep(duration + 0.5)

def backUpTurnAroundEmpty():
  global gBeepQueue, gWheelQueue
  global gKillBehavior
  for robot in gRobotList:
    gWheelQueue.put([-100, -100, 0.5])

    duration = randint(5, 10) / 10.0
    gWheelQueue.put([-WHEEL_SPEED ,WHEEL_SPEED, duration])
    time.sleep(duration + 0.5)

def turnAroundEmpty():
  global gBeepQueue, gWheelQueue
  global gKillBehavior
  gBeepQueue.put([30, 4, 4, 0.5])
  for robot in gRobotList:
    duration = randint(5, 10) / 10.0
    gWheelQueue.put([-WHEEL_SPEED ,WHEEL_SPEED, duration])
    time.sleep(duration + 0.5)


def hitBorder():
  for robot in gRobotList:
    left = robot.get_floor(0)
    right = robot.get_floor(1)
    return (left <= FLOOR_BLACK or right <= FLOOR_BLACK)



def objectAligned():
  for robot in gRobotList:
    left = robot.get_proximity(0)
    right = robot.get_proximity(1)
    alignment = abs(left-right)
    return alignment <= ALIGN_SKEW_THRESHOLD

def pushObject():
  global gBeepQueue, gWheelQueue
  global gKillBehavior

  while not gKillBehavior:
    for robot in gRobotList:
      left = robot.get_proximity(0)
      right = robot.get_proximity(1)
      correction = (right - left) / 2
      gWheelQueue.put([WHEEL_SPEED + correction, WHEEL_SPEED - correction, 0.1])
      '''
      if left > right:
        gWheelQueue.put([int(WHEEL_SPEED * 0.75), WHEEL_SPEED, 0.1])
      elif left < right:
        gWheelQueue.put([WHEEL_SPEED, int(WHEEL_SPEED * 0.75), 0.1])
      else:
        gWheelQueue.put([WHEEL_SPEED, WHEEL_SPEED, 0.1])
      '''
      print 'left proximity: ', left, ' right:', right
      time.sleep(0.1)

def foundObject():
  for robot in gRobotList:
    left = robot.get_proximity(0)
    right = robot.get_proximity(1)
    return (left > FOUND_OBJ_THRESHOLD or right > FOUND_OBJ_THRESHOLD)

def alignObject():
  global gBeepQueue, gWheelQueue
  global gKillBehavior
  gWheelQueue.put([0, 0, 0])
  gBeepQueue.put([50, 3, 3, 0.2])

  while not gKillBehavior:
    for robot in gRobotList:
      left = robot.get_proximity(0)
      right = robot.get_proximity(1)
      correction = abs((right - left) / 2)
      if left > right:
        gWheelQueue.put([10, 10 + correction, 0.1])
      elif left < right:
        gWheelQueue.put([10+correction, 10, 0.1])
      time.sleep(0.1)


def onWhite():
  for robot in gRobotList:
    left = robot.get_floor(0)
    right = robot.get_floor(1)
    if (left > FLOOR_WHITE and right > FLOOR_WHITE):
      return True
  return False

def waitForWhite():
  global gBeepQueue, gWheelQueue
  global gKillBehavior
  #gWheelQueue.put([0, 0, 0])
  gWheelQueue.put([ 1, 5, FLAG_LINETRACE])

  while (not gKillBehavior):
    gBeepQueue.put([30, 4, 4, 0.1])
    gBeepQueue.put([0, 0, 0, 0.1])
    time.sleep(0.2)

def searchFwd():
  global gBeepQueue, gWheelQueue
  global gKillBehavior

  while not gKillBehavior:
    #fwdInterval = randint(10, 50) / 10.0
    skew = randint(-WHEEL_SPEED/5, WHEEL_SPEED/5)
    #gWheelQueue.put([WHEEL_SPEED+skew, WHEEL_SPEED-skew, fwdInterval])
    gWheelQueue.put([WHEEL_SPEED+skew, WHEEL_SPEED-skew, 0.1])
    time.sleep(0.1)
  '''
  while(True):
    if (note > 88):
      note = (note % 88) + 1
    gBeepQueue.put([note, 1, 1, 0.05])
    time.sleep(0.05)
    note += 12
  '''


def fsm_init():
  global FSM
  FSM = StateMachine()

  State_Start = FSM.add_state("Start")
  State_WaitForWhite = FSM.add_state("WaitForWhite")
  State_SearchFwd = FSM.add_state("SearchFwd")
  State_BackUpTurnAroundEmpty = FSM.add_state("BackUpTurnAroundEmpty")
  State_AlignObject = FSM.add_state("AlignObject")
  State_PushObject = FSM.add_state("PushObject")
  State_BackUp = FSM.add_state("BackUp")
  State_CheckSuccess = FSM.add_state("CheckSuccess")
  State_TurnAroundEmpty = FSM.add_state("TurnAroundEmpty")
  State_TurnAroundFull = FSM.add_state("TurnAroundFull")
  State_Done = FSM.add_state("Done")

  FSM.set_start("Start")
  FSM.set_current("Start")

  State_Start.add_transition("WaitForWhite", lambda: True, waitForWhite)

  State_WaitForWhite.add_transition("SearchFwd", onWhite, searchFwd)

  State_SearchFwd.add_transition("AlignObject", foundObject, alignObject)

  State_SearchFwd.add_transition("BackUpTurnAroundEmpty", hitBorder, backUpTurnAroundEmpty)

  State_BackUpTurnAroundEmpty.add_transition("SearchFwd", onWhite, searchFwd)

  State_AlignObject.add_transition("PushObject", objectAligned, pushObject)

  State_PushObject.add_transition("BackUp", hitBorder, backUp)

  State_BackUp.add_transition("CheckSuccess", backUpDelay, checkSuccess)

  State_CheckSuccess.add_transition("TurnAroundFull", foundObjectClose, turnAroundFull)
  State_CheckSuccess.add_transition("TurnAroundEmpty", lambda: not foundObjectClose(), turnAroundEmpty)

  State_TurnAroundEmpty.add_transition("SearchFwd", onWhite, searchFwd)

  State_TurnAroundFull.add_transition("Done", clearedTarget, done)
  State_TurnAroundFull.add_transition("SearchFwd", lambda: onWhite() and not clearedTarget(), searchFwd)


def display_target():
  global gCanvas
  global gProximityBarLeft, gProximityBarRight
  global gQuit

  if (not gProximityBarLeft or not gProximityBarRight):
    createProximityBars()

  while not gQuit:
    if (len(gRobotList) > 0):
      for robot in gRobotList:
        updateProximityBars(robot.get_proximity(0), robot.get_proximity(1))
        updateHamsterBox(robot.get_floor(0), robot.get_floor(1))

    time.sleep(0.1)

def wheel_target():
  global gWheelQueue
  global gQuit

  #Queue element format: [Left wheel, Right wheel, duration]
  #If duration == 0, holds indefinitely until next element dequeued
  while not gQuit:
    movement = gWheelQueue.get(True)
    print 'movement: ', movement
    for robot in gRobotList:
      robot.set_wheel(0, movement[0])
      robot.set_wheel(1, movement[1])
      if (movement[2] == FLAG_LINETRACE):
        robot.set_wheel(0, 0)
        robot.set_wheel(1, 0)
        robot.set_line_tracer_mode_speed(movement[0], movement[1])
			else:
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


def StartRace(event=None):
  global monitor_thread, dispatch_thread
  global display_thread, beep_thread, wheel_thread
  global gNumCleared

  if (len(gRobotList) > 0):

    # display thread
    gNumCleared = 0
    display_thread = threading.Thread(target=display_target)
    display_thread.daemon = True
    display_thread.start()

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

    wheel_thread = threading.Thread(target = wheel_target)
    wheel_thread.daemon = True
    wheel_thread.start()


def startrace(self):
    gEventQueue.put( "start", StartRace )
    print "start button pressed: start monitor and dispatch threads"

def stop(self):
    print "stop threads button pressed"
    gEventQueue.put( "done", done )

def draw_track():
  global gCanvas, frame
  trackoriginx = 40
  trackoriginy = 40
  trackwidth = 400 * 1.5
  trackheight = 400
  trackcutamount = 100

  rect = gCanvas.create_polygon(trackoriginx + trackcutamount, trackoriginy, \
      trackoriginx + trackwidth - trackcutamount, trackoriginy, \
      trackoriginx + trackwidth, trackoriginy + trackcutamount, \
      trackoriginx + trackwidth, trackoriginy + trackheight - trackcutamount, \
      trackoriginx + trackwidth - trackcutamount, trackoriginy + trackheight, \
      trackoriginx + trackcutamount, trackoriginy + trackheight, \
      trackoriginx, trackoriginy + trackheight - trackcutamount, \
      trackoriginx, trackoriginy + trackcutamount, \
      outline="black", fill="white", width=2)

  line = gCanvas.create_line(trackoriginx, trackheight + 40, trackoriginx + 20, trackheight + 40, width = 3)

  text = gCanvas.create_text(trackoriginx, trackheight + 60, font="Purisa",
            text="28 mm")

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
        startRaceButton.bind('<Button-1>', StartRace)

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
    def __init__(self, comm, m, gCanvas, vrobot, robot_i=0, keyBindings=['w','s','a','d','x'])
        self.gMaxRobotNum = 1
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
            if self.gRobotList is not None:
                robot = self.gRobotList[self.robot_i]

                #--- update wheel balance
                self.gRobotList[self.robot_i].set_wheel_balance(3)

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

def draw_virtual_world(virtual_world, joystick):
    time.sleep(1) # give time for robot to connect.
    while not gQuit:
        if joystick.gRobotList is not None:
            virtual_world.draw_robot()
            virtual_world.draw_prox("left")
            virtual_world.draw_prox("right")
            virtual_world.store_prox()
            virtual_world.draw_floor("left")
            virtual_world.draw_floor("right")
        time.sleep(0.1)

def main():
  global gRobotList, gQuit
  global gCanvas, frame, gHamsterBox
  global display_thread, monitor_thread, dispatch_thread
  global gBeepQueue, gWheelQueue, drawQueue

  comm = RobotComm(gMaxRobotNum)
  comm.start()
  print 'Bluetooth started'

  gRobotList = comm.robotList

  monitor_thread = False
  dispatch_thread= False

  display_thread = False

  # frame = tk.Tk()
  # CANVAS_SIZE = 800
  # gCanvas = tk.Canvas(frame, bg="white", width=CANVAS_SIZE, height=CANVAS_SIZE)
  # gCanvas.pack(expand=1, fill='both')

  # draw_track()

  # # UI
  # gHamsterBox = gCanvas.create_rectangle(BOX_X, BOX_Y, BOX_X + BOX_SIZE,
  #     BOX_Y + BOX_SIZE, fill="white", width=BORDER_WIDTH)
  # cleanButton = tk.Button(frame, text="Start")
  # cleanButton.pack()
  # cleanButton.bind('<Button-1>', StartClean)
  # tk.Button(frame, text="Quit", command=quit).pack()
  # tk.Button(frame, text="Start Race", command=startrace).pack()
  # tk.Button(frame, text="Stop Threads", command=stop).pack()

  # create virtual robot data object
  vrobot = virtual_robot()
  pi4 = 3.1415 / 4
  vrobot.set_robot_a_pos(pi4*2, -540, +340)

  # create UI
  frame = tk.Tk()
  canvas_width = 700 # half width
  canvas_height = 380 # half height
  gCanvas = tk.Canvas(frame, bg="white", width=canvas_width*2, height=canvas_height*2)
  draw_track()

  # keyboard input
  joystick = Joystick(comm, frame, gCanvas, vrobot)
  poly_points = [0,0,0,0,0,0,0,0]
  joystick.vrobot.poly_id = gCanvas.create_polygon(poly_points, fill='blue') #robot
  joystick.vrobot.prox_l_id = gCanvas.create_line(0,0,0,0, fill="red") #prox sensors  ---- here
  joystick.vrobot.prox_r_id = gCanvas.create_line(0,0,0,0, fill="red")
  joystick.vrobot.floor_l_id = gCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
  joystick.vrobot.floor_r_id = gCanvas.create_oval(0,0,0,0, outline="white", fill="white")

  time.sleep(1)

  update_vrobot_thread = threading.Thread(target=joystick.update_virtual_robot)
  update_vrobot_thread.daemon = True
  update_vrobot_thread.start()

  # virtual world UI
  drawQueue = Queue.Queue(0)
  vWorld = virtual_world(drawQueue, joystick, joystick.vrobot, gCanvas, canvas_width, canvas_height)
  rectA = [-560, 300, -520, 260]
  vWorld.add_obstacle(rectA)

  draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld, joystick))
  draw_world_thread.daemon = True
  draw_world_thread.start()

  gui = VirtualWorldGui(vWorld, frame)

  gui.drawGrid()
  gui.drawMap()

  gCanvas.after(200, gui.updateCanvas, drawQueue)
  frame.mainloop()

  for robot in joystick.gRobotList:
      robot.reset()
  comm.stop()
  comm.join()

  frame.mainloop()

  print 'Cleaning up'
  gQuit = True

  for robot in gRobotList:
    robot.reset()
  time.sleep(1.0)
  comm.stop()
  comm.join()

  print 'Terminated'

if __name__ == "__main__":
    main()
