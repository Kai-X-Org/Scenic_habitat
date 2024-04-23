import magnum as mn
import numpy as np
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
from scenic.simulators.habitat.utils import scenic_to_habitat_map
from scenic.core.vectors import Vector
import math
import time

behavior NavToObj(obj):
    x, y, z = obj.position
    do RobotNav(x=x, y=y, z=z)

behavior RobustHumanNav(x=0, y=0, z=0, sample_radius=1.0):
    x, y, z, _, _, _ = scenic_to_habitat_map((x, y, z, 0, 0, 0))
    navigable = simulation().sim.pathfinder.is_navigable(np.array([x, y, z]))
    print(f"Point Navigable: {navigable}")
    if navigable:
        do HumanNav(x=x, y=y, z=z)
    else:
        new_point = simulation().sim.pathfinder.get_random_navigable_point_near(circle_center=np.array([x, y, z]),
                                                                                    radius=1.0)
        x, y, z = new_point
        print(f'new_point: {new_point}')
        do HumanNav(x=x, y=y, z=z)


behavior SpotPickUp(box=None):
    # start_pos = np.array(self._articulated_agent.arm_joint_pos)
    raise_pos = np.array([0.0, -3.14, 0.00, 1.57, 0.0, 0.0, 0.0]) # forarm raise
    # delta_pos = (raise_pos - start_pos)/100
    # for _ in range(100):
        # new_pos = list(start_pos + delta_pos)
        # take SpotMoveArmAction(arm_ctrl_angles=new_pos)
        # start_pos = np.array(self._articulated_agent.arm_joint_pos)
    do MoveToJointAngles(raise_pos)
    
    # raise_pos = [0.0, -1.57, 0.0, 1.57, 0.0, 0.0, 0.0] # shoulder raise
    raise_pos = [0.0, -1.0, 0.0, 1.57, 0.0, 0.0, 0.0] # shoulder raise
    do MoveToJointAngles(raise_pos)
    
    spot_ee_pos = self.ee_pos
    box_pos = box.position
    diff_pos = box_pos - spot_ee_pos
    diff_norm = np.linalg.norm(np.array([diff_pos[0], diff_pos[1], diff_pos[2]]))  
    print(f"pos_difference norm: {diff_norm}")
    if diff_norm < 0.25:
        take SnapToObjectAction(box)

    raise_pos = np.array([0.0, -3.14, 0.00, 3.14, 0.0, 0.0, 0.0]) # forarm raise
    do MoveToJointAngles(raise_pos, steps=50)



behavior NavToHuman(human):
    position = human.position
    x, y, z = position[0], position[1], position[2]
    do RobotNav(x, y, z)

behavior GrabAndNav(box, human):
    do SpotPickUp(box=box)
    do NavToHuman(human)

behavior MoveToJointAngles(joint_angles, steps=50):
    start_pos = np.array(self._articulated_agent.arm_joint_pos)
    delta_pos = (joint_angles - start_pos)/steps
    for _ in range(steps):
        new_pos = list(start_pos + delta_pos)
        take SpotMoveArmAction(arm_ctrl_angles=new_pos)
        start_pos = np.array(self._articulated_agent.arm_joint_pos)
        
behavior ReachHand():
    # do HumanReach(x=-1, y=-1, z=0.5, index_hand=0)
    do HumanReach(x=-0.5, y=-0.5, z=0.5, index_hand=0)
    # do HumanReach(x=1, y=0, z=0.5, index_hand=1)

behavior ReachHandAndWalk(walk_position, reach_position):
    try:
        reach_x = reach_position[0]
        reach_y = reach_position[1]
        reach_z = reach_position[2]
        do HumanReach(x=x, y=y, z=z, index_hand=0)
        while True:
            wait
    interrupt when ego 

bed = RectangularRegion((0.3, -6.0, 0.63), 0, 1.0, 1.0) # final defined bed width
box = new GelatinBox on (0.12, -5.5, 0.61)
# human = new Female_0 at (-5.0, -3.0, 0)
human = new Female_0 at (-0.5, -4.8, 0), with yaw -90 deg, with behavior ReachHandAndWalk()
ego = new SpotRobot at (-0.9, -5.5, 0), with behavior SpotPickUp(box=box)

