import behavior
import time

class Behavior(behavior.RobotBehavior):
    def __init__(self, name, robotList, default_speed = 30):
        super(Behavior, self).__init__(name, robotList)
        self._default_speed = default_speed 

    def behavior_loop(self):
        print self._name, "behavior_loop() starts!"
        speed = self._default_speed
        while (not self._bQuit):
            for robot in self._robotList:
                #print self._name, "behavior_loop(): time = ", time.time(), "sec"
                robot.set_wheel(0, speed)
                robot.set_wheel(1, speed)
                speed = -speed
                time.sleep(1.0)

            time.sleep(0.001)
        print self._name, "behavior_loop() finished!"

