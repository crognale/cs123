import behavior
import time

class Behavior(behavior.RobotBehavior):
    def __init__(self, name, robotList):
        super(Behavior, self).__init__(name, robotList)

    def behavior_loop(self):
        print self._name, "behavior_loop() starts!"
        color = 0
        while (not self._bQuit):
            for robot in self._robotList:
                #print self._name, "behavior_loop(): time = ", time.time(), "sec"
                robot.set_led(0, color)
                robot.set_led(1, color)
                if color > 7:
                    color = 0
                else:
                    color += 1
                time.sleep(1.0)

            time.sleep(0.001)
        print self._name, "behavior_loop() finished!"
