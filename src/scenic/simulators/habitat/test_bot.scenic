import magnum as mn
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
import math
import time

behavior OpenGripper():
    take OpenGripperAction()
    t0 = time.time()
    t1 = t0
    while t1 - t0 < 3:
        wait
        t1 = time.time()
    take CloseGripperAction()
    while t1 - t0 < 3:
        wait
        t1 = time.time()
    terminate
    
behavior SnapToObject(target_object):
    t0 = time.time()
    t1 = t0
    take SnapToObjectAction(target_object)
    while t1 - t0 < 3:
        wait
        t1 = time.time()
    terminate
ego = new SpotRobot at (-1.5, -5.5, 0), with yaw -35 deg, with behavior OpenGripper()
