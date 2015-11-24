import time

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
      for i in range(0,4):
        state = self.states[self.currentState] 
        print "current state", state.name
        #print "transitions", state.transitions
        for transition in state.transitions:
          print "checking transition to", transition
          if transition[1](): #condition satisfied
            if (transition[2] != False):
              transition[2]()
            self.currentState = transition[0]
        time.sleep(1)

def click_a():   #condition from B to A
  print "click a"
  return True

def click_b():   #condition from A to B
  print "click b"
  return True

def moving_to_A(): #callback function when transition from B to A
  print "moving to A"

def moving_to_B():
  print "moving to B" #callback function when transition from A to B

def main():
    MyStateMachine = StateMachine()
    print "created a state machine"

    State_A = MyStateMachine.add_state("A")
    State_B = MyStateMachine.add_state("B")
    MyStateMachine.set_start("A")
    MyStateMachine.set_current("A")

    State_A.add_transition("B", click_b, moving_to_B)
    State_B.add_transition("A", click_a, moving_to_A)

    MyStateMachine.run()

if __name__ == "__main__":
    main()
