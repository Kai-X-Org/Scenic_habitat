import magnum as mn
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
from scenic.core.vectors import Vector
import math
import time

behavior GoRel(x=0, y=0, z=0, rot=0, num_steps=100):
    # agent = simulation().sim.agents_mgr[self._agent_id].articulated_agent
    dx = x/num_steps
    dy = y/num_steps
    dz = z/num_steps
    
    # print('taking action!')
    waited = False
    try:
        for _ in range(num_steps):
            print('taking action!')
            take GoRelDeltaAction(self, dx, dy, dz)
            
            print(self.position)
            # take GoRelDeltaAction()
        # if self._articulated_agent_type == 'FetchRobot':
        # terminate
    interrupt when (self.distanceToClosest(KinematicHumanoid) < 1.5 and not waited):
        waited = True
        t0 = time.time()
        t1 = t0
        while t1 - t0 < 1:
            wait
            t1 = time.time()
        print('finish scene')



behavior HumanGo(x=0, y=0, z=0, num_steps=100):
    step_count = 0
    start_position = self.position
    try:
        while step_count < num_steps and \
                not np.isclose((self.position - start_position).norm(), Vector(x, y, z).norm(), atol=0.1):
            take HumanGoAction(x, y, z) # TODO temporary implementation
            print(f"Scenic position: {self.position}")
            step_count += 1

        take HumanStopAction()

    interrupt when (self.distanceToClosest(FetchRobot) < 1.5):
        take HumanStopAction()
        t0 = time.time()
        t1 = t0
        while t1 - t0 < 3:
            wait
            t1 = time.time()
        terminate

    print('finished walking')
    print(f"target: {x, y, z}")
    # print(f"pos_delta: {pos_delta}")
    t0 = time.time()
    t1 = t0
    while t1 - t0 < 3:
        wait
        t1 = time.time()
    print('finish scene')
    terminate

# human = new Female_0 at (-1.5, -3.5, 0), with yaw -90 deg,  with behavior HumanGo(y=1)
ego = new FetchRobot at (-1.8, Range(-6.5, -4.5), 0), with behavior GoRel(y=4.0)
human = new Female_0 at (1.5, -4.5, 0), with yaw 90 deg,  with behavior HumanGo(x=-4)
