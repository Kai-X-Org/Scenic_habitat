model scenic.simulators.habitat.model
from scenic.simulators.habitat.model import *
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.utils import scenic_to_habitat_map
import magnum as mn
import numpy as np

behavior GoToLookAt(obj):
    obj_id = obj._object_id
    object_trans = obj._managed_rigid_object.translation

    object_agent_vec = self._articulated_agent.base_pos - object_trans
    object_agent_vec.y = 0
    dist_agent_object = object_agent_vec.length()

    agent_displ = np.inf
    agent_rot = np.inf
    
    while agent_displ > 1e-9 or agent_rot > 1e-9:
        print("WALKING")
        prev_rot = self._articulated_agent.base_rot
        prev_pos = self._articulated_agent.base_pos

        take HumanoidNavLookAtAction(object_trans)

        cur_rot = self._articulated_agent.base_rot
        cur_pos = self._articulated_agent.base_pos
        agent_displ = (cur_pos - prev_pos).length()
        agent_rot = np.abs(cur_rot - prev_rot)

    # wait
    for _ in range(20):
        wait


behavior GoRel(x=0, y=0, z=0, rot=0, num_steps=100):
    # agent = simulation().sim.agents_mgr[self._agent_id].articulated_agent
    dx = x/num_steps
    dy = y/num_steps
    dz = z/num_steps
    
    # print('taking action!')
    for _ in range(num_steps):
        # print('taking action!')
        take GoRelDeltaAction(self, dx, dy, dz)
        
        print(self.position)
        # take GoRelDeltaAction()
    # if self._articulated_agent_type == 'FetchRobot':
        terminate

behavior MoveAndBack(x=0, y=0, z=0, num_steps=100):
    try:
        do GoRel(x=x, y=y, z=z, num_steps=100)
    interrupt when (self.distanceToClosest(KinematicHumanoid) < 1.5):
        do GoRel(x=-x/2, y=-y/2, z=-z/2, num_steps=100)
        terminate

behavior HumanGo(x=0, y=0, z=0, num_steps=100):

    step_count = 0
    # pos_delta = Vector(x, y, z)
    
    # x, y, z = self.position + pos_delta
    start_position = self.position
    while step_count < num_steps and \
            not np.isclose((self.position - start_position).norm(), Vector(x, y, z).norm(), atol=0.1):
        take HumanGoEnvAction(x, y, z) # TODO temporary implementation
        print(f"Scenic position: {self.position}")
        step_count += 1

    take HumanStopAction()

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

behavior HumanReach(x=0, y=0, z=0, index_hand=0):
    arr = np.array([x, y, z])
    arr = (arr - 0.5) * 0.1
    
    for _ in range(100):
        take HumanReachAction(x=arr[0], y=arr[1], z=arr[2], index_hand=index_hand)

behavior GoAndReach(reach_x=0, reach_y=0, reach_z=0, move_x=0, move_y=0, move_z=0, index_hand=0):
    do HumanReach(x=reach_x, y=reach_y, z=reach_z, index_hand=index_hand)
    do HumanGo(x=move_x, y=move_y, z=move_z)

behavior RobotNav(x=0, y=0, z=0):
    for _ in range(100):
        take OracleCoordAction(x, y, z)
    terminate

behavior MoveSpotArm():
    take SpotMoveArmAction(arm_ctrl_angles=[1.57, -1.57, 0.0, 1.57, 0.0, 0.0, 0.0])
    # take SpotMoveArmAction(arm_ctrl_angles=[0.00, -3.14, 0.0, 3.00, 0.0, 0.0, 0.0]) # default
    t0 = time.time()
    t1 = t0
    while t1 - t0 < 1.5:
        wait
        t1 = time.time()
    print('finish scene')
    take SpotMoveArmAction()

behavior HumanNav(x=0, y=0, z=0):
    for _ in range(100):
        take HumanoidNavAction(x, y, z)
    terminate

behavior FetchReach(x=0, y=0, z=0, frame='world'):
    # take FetchReachAction(x=x, y=y, z=z)
    for _ in range(100):
        take FetchReachAction(x=x, y=y, z=z, frame='world')


behavior FetchDumReach(x=0, y=0, z=0, frame='world'):
    x, y, z, _, _, _ = scenic_to_habitat_map((x, y, z, 0, 0, 0))
    # z -= 100
    base_transform = self._articulated_agent.base_transformation 
    des_ee_pos = base_transform.inverted().transform_point(mn.Vector3(x, y, z))
    # print(f"base_transform: {np.array(base_transform)}")
    # print(f"inv_base_transform: {np.array(base_transform.inverted())}")
    # print(f"pdt of mat {np.array(base_transform) @ np.array(base_transform.inverted())}")
    # print(f"desired ee pos: {des_ee_pos}")
    # print(f"ee_transform:{self._articulated_agent.ee_transform()}")
    # des_joint_pos = self._ik_helper.calc_ik(np.array([x, y, z]))
    current_joint_pos = self._articulated_agent.arm_joint_pos
    self._ik_helper.set_arm_state(current_joint_pos)
    des_joint_pos = self._ik_helper.calc_ik(np.array(des_ee_pos))
    # print(f"desired joint pos : {des_joint_pos}")

    # print("current joint pos: ",np.array(current_joint_pos))
    joint_pos_diff = np.array(des_joint_pos) - np.array(current_joint_pos)
    # print("joint_pos_diff = ", joint_pos_diff)
    joint_pos_delta = joint_pos_diff/100
    for _ in range(100):
        new_joint_pos = list(current_joint_pos + joint_pos_delta)
        # print("new joint pos: ", new_joint_pos)
        # print("des joint pos:", des_joint_pos)
        take FetchSetJointAction(new_joint_pos)
        current_joint_pos = self._articulated_agent.arm_joint_pos
        # print("current_joint_position: ", current_joint_pos)
        joint_pos_diff = np.array(des_joint_pos) - np.array(current_joint_pos)
        # print("current_joint_pos_diff = ", joint_pos_diff)
    
