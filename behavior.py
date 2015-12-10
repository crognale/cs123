import time

class RobotBehavior(object):
    def __init__(self, name, robotList):
        self._name = name
        self._robotList = robotList
        self._bQuit = False

    def get_name(self):
        return self._name

    def set_bQuit(self, bQuit):
        self._bQuit = bQuit
        print self._name, "RobotBehavior.set_bQuit():", bQuit

    def behavior_loop(self):
        while (self._bQuit):
            for robot in self._robotList:
                print self._name, "RobotBehavior.behavior_loop(): time =", time.time(), "sec"
            time.wait(0.001)
    
