import magnum as mn
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
import math
import time
import numpy as np
bed_center = Vector(0.5, -6.0, 0.4)

bed = RectangularRegion(bed_center, 1.57, 1.0, 1.0)
sampling_height = 1.0
sampling_radius = 0.7
sampling_space = BoxRegion(position=bed_center + Vector(0, 0, sampling_height), 
                           dimensions=Vector(sampling_radius, sampling_radius, sampling_radius))
ego = new SpotRobot at (-1.5, -5.5, 0), with yaw -35 deg
# hammer = new Hammer on ego
clutter_list = list()
for i in range(40):
    # point = new Point in sampling_space
    # new_cube = new MasterChef in sampling_space #, with yaw Range(0, 360) deg
    new_box = new GelatinBox in sampling_space #, with yaw Range(0, 360) deg
