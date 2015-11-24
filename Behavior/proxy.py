import behavior
import time

class Behavior(behavior.RobotBehavior):
    def __init__(self, name, robotList, max_value, detecting_period, queue):
        super(Behavior, self).__init__(name, robotList)
        self._period = detecting_period
        self._queue = queue
        self._max_value = max_value

    def behavior_loop(self):
        print self._name, "behavior_loop() starts!"
        while (not self._bQuit):
            for robot in self._robotList:
                print self._name, "behavior_loop(): time = ", time.time(), "sec"
                left = robot.get_proximity(0)
                right = robot.get_proximity(1)
                print left, right

                left = self._max_value - left
                right = self._max_value - right

                #draw graphics
                elem = {}
                elem[0] = left
                elem[1] = right
                self._queue.put(elem)
            time.sleep(self._period)
        print self._name, "behavior_loop() finished!"
