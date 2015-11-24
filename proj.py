from threading import Thread
import threading
import time
from HamsterAPI.comm_ble import RobotComm
import Tkinter as tk
import Queue
from random import randint

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

gProximityBarLeft = None
gProximityBarRight = None
gHamsterBox = None 

gBeepQueue = Queue.Queue()
gWheelQueue = Queue.Queue()

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

    def run(self):
			global gKillBehavior

			behavior_thread = None
			while(not gQuit):
				state = self.states[self.currentState] 
				print "current state", state.name
				for transition in state.transitions:
					if transition[1](): #condition satisfied
						#Stop current behavior
						gKillBehavior = True
						if (behavior_thread):
							behavior_thread.join()
						gKillBehavior = False
						if (transition[2] != False):
							#Start new behavior
							behavior_thread = threading.Thread(target=transition[2])
							behavior_thread.daemon = True
							behavior_thread.start()
						self.currentState = transition[0]
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
	gWheelQueue.put([0, 0, 0])

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


def fsm_target():
	global gQuit
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

	FSM.run()


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
			if (movement[2] > 0):
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


def StartClean(event=None):
	global display_thread, fsm_thread, beep_thread, wheel_thread
	global gNumCleared

	if (len(gRobotList) > 0):
		gNumCleared = 0
		display_thread = threading.Thread(target=display_target)
		display_thread.daemon = True
		display_thread.start()

		fsm_thread = threading.Thread(target=fsm_target)
		fsm_thread.daemon = True
		fsm_thread.start()

		beep_thread = threading.Thread(target = beep_target)
		beep_thread.daemon = True
		beep_thread.start()

		wheel_thread = threading.Thread(target = wheel_target)
		wheel_thread.daemon = True
		wheel_thread.start()

def main():
	global gRobotList, gQuit
	global gCanvas, frame, gHamsterBox
	global display_thread, fsm_thread
	global gBeepQueue, gWheelQueue

	comm = RobotComm(gMaxRobotNum)
	comm.start()
	print 'Bluetooth started'

	gRobotList = comm.robotList


	fsm_thread = False

	display_thread = False
	frame = tk.Tk()
	gCanvas = tk.Canvas(frame, bg="white", width=CANVAS_SIZE, height=CANVAS_SIZE)
	gCanvas.pack(expand=1, fill='both')
	gHamsterBox = gCanvas.create_rectangle(BOX_X, BOX_Y, BOX_X + BOX_SIZE,
			BOX_Y + BOX_SIZE, fill="white", width=BORDER_WIDTH)

	cleanButton = tk.Button(frame, text="Start")
	cleanButton.pack()
	cleanButton.bind('<Button-1>', StartClean)

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
