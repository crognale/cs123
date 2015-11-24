'''
/* =======================================================================
   eventfsm.py

   - event-drven fsm
   - create monitor events loop
   ========================================================================*/
'''

import time
import threading
import Queue
import hamster_main
from hamster_main import *

quit = False
event_queue = None
monitorevents_timesleep = 0.1

class Event:
  def __init__(self, name):
    self.name = name


class State:
    def __init__(self):
      self.name = ''
      self.transitions = {}

    def add_transition(self, toState, callback):
      self.transitions[toState] = callback

# event fsm: handle events in 'run' function
class EventFsm:
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

    # handler: the handler for events is implemented as transition callback fcns that occur between states in the fsm
    def run(self):
      while not quit:
        if event_queue:
          event = event_queue.get()
          if event.name in self.currentState.transitions:
            self.currentState.transitions[event]()  # call transition function to exert change in robot
            self.set_current(event.name)
        time.sleep(1)

def ready_to_inlane():

  global comm
  if comm:
    robotList = comm.get_robotList()

  speed = self._default_speed
  while (not self._bQuit):
    for robot in self._robotList:
      #print self._name, "behavior_loop(): time = ", time.time(), "sec"
      mode = 0x01
      speed = 7
      robot.set_line_tracer_mode_speed(mode, speed) # 0x11 ~ 0x6A (0: off), speed 7(100%)
      # time.sleep(1.0)

      print "floor sensors: ", robot.get_floor(0), robot.get_floor(1)

    time.sleep(0.001)
  print "transition ready -> inlane"

def inlane_to_finish():
  print "transition inlane -> finish"

# dispatcher: place events into queue
def monitor_events():
  global event_queue
  global comm
  events = ["ready", "inlane", "finish"]

  if comm:
    robotList = comm.get_robotList()

    for robot in self._robotList:
      if robot.get_floor(0) < 50:
        event_queue.put("inlane")

def build_states(fsm):

  # states: ready, inlane, finish
  state_ready = fsm.add_state("ready")
  state_inlane = fsm.add_state("inlane")
  state_finish = fsm.add_state("finish")

  # ready state transitions to inlane
  state_ready.add_transition("inlane", ready_to_inlane)

  # inlane state transitions to finish
  state_inlane.add_transition("finish", inlane_to_finish)

  # finish state has no transitions

