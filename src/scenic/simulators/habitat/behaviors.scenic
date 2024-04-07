model scenic.simulators.habitat.model
from scenic.simulators.habitat.model import *
from scenic.simulators.habitat.actions import *


behavior GoToLookAt(obj):
    obj_id = obj._object_id
    object_trans = simulation().rigid_obj_mgr.get_object_by_id(obj_id)
    delta = 2.0

    object_agent_vec = self._articulated_agent.base_pos - object_trans
    object_agent_vec.y = 0
    dist_agent_object = object_agent_vec.length()

    agent_displ = np.inf
    agent_rot = np.inf
    prev_rot = self._articulated_agent.base_rot
    prev_pos = self._articulated_agent.base_pos
    
    while agent_displ > 1e-9 or agent_rot > 1e-9:
        prev_rot = self._articulated_agent.base_rot
        prev_pos = self._articulated_agent.base_pos

        take HumanoidNavLookAtAction(object_trans)

        cur_rot = env.sim.articulated_agent.base_rot
        cur_pos = env.sim.articulated_agent.base_pos
        agent_displ = (cur_pos - prev_pos).length()
        agent_rot = np.abs(cur_rot - prev_rot)

    # wait
    for _ in range(20):
        wait

