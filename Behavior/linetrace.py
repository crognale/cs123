import behavior
import time

class Behavior(behavior.RobotBehavior):
    def __init__(self, name, robotList, default_speed = 30):
        super(Behavior, self).__init__(name, robotList)
        self._default_speed = default_speed 

    def behavior_loop(self):
        print self._name, "behavior_loop() starts!"

        print self._name, "behavior_loop() finished!"

