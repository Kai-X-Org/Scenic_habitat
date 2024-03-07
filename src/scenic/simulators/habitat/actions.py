import habitat_sim
import magnum as mn
import warnings
from habitat.tasks.rearrange.rearrange_sim import RearrangeSim
warnings.filterwarnings('ignore')
from habitat_sim.utils.settings import make_cfg
from matplotlib import pyplot as plt
from habitat_sim.utils import viz_utils as vut
from omegaconf import DictConfig
import numpy as np
from habitat.articulated_agents.robots import FetchRobot
from habitat.config.default import get_agent_config
from habitat.config.default_structured_configs import ThirdRGBSensorConfig, HeadRGBSensorConfig, HeadPanopticSensorConfig
from habitat.config.default_structured_configs import SimulatorConfig, HabitatSimV0Config, AgentConfig
from habitat.config.default import get_agent_config
from habitat.tasks.rearrange.actions.actions import HumanoidJointAction
import habitat
from habitat_sim.physics import JointMotorSettings, MotionType
from omegaconf import OmegaConf

from scenic.core.simulators import *


class GoRelDeltaAction(Action):

    def __init__(self, obj, dx=0, dy=0, dz=0, rot=0):
        self.pos_delta = mn.Vector3(dx, dy, dz)
        self.art_agent = obj._articulated_agent
        #TODO add rotation delta
    
    def applyTo(self, obj, sim):
        # art_agent = sim.sim.articulated_agent
        x, y, z = self.pos_delta
        x, y, z, _, _, _ = sim.scenicToHabitatMap((x, y, z,0,0,0))
        self.pos_delta = np.array([x, y, z])
        if obj._articulated_agent_type == 'KinematicHumanoid':
            rel_pose = self.art_agent.base_pos + self.pos_delta
            obj._humanoid_controller.calculate_walk_pose(rel_pose)
            new_pose = obj._humanoid_controller.get_pose()
            joint_action = obj._humanoid_joint_action
            # print('action arg prefix!!!:', joint_action._action_arg_prefix)
            key = joint_action._action_arg_prefix + 'human_joints_trans'
            # key = f'agent_{obj._agent_id}' + '_human_joint_trans'
            # print('KEY:', key)
            arg_dict = dict()
            arg_dict[key] = new_pose
            # print('ARG_DICT', arg_dict)
            joint_action.step(**arg_dict)

        else:
            self.art_agent.base_pos = self.art_agent.base_pos + self.pos_delta
        return

class HumanGoAction(Action):
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def applyTo(self, obj, sim):
        self.art_agent = obj._articulated_agent
        x, y, z, _, _, _ = sim.scenicToHabitatMap((self.x, self.y, self.z,0,0,0))
        # print(f'Moving {(x, y, z)}')

        print("current BASE POS:", self.art_agent.base_pos)
        # self.pos_delta = np.array([x, y, z])
        # rel_pose = mn.Vector3(self.art_agent.base_pos + self.pos_delta) 
        rel_pose = mn.Vector3(x, y, z)

        print('rel_pose', rel_pose)
        obj._humanoid_controller.reset(obj._articulated_agent.base_transformation) # probelm, likely relative to human frame?
        obj._humanoid_controller.calculate_walk_pose(rel_pose)

        print("BASE POS1:", obj._articulated_agent.base_pos)
        print("BASE_TRANSFORMATION:", obj._articulated_agent.base_transformation)

        human_joints_trans = obj._humanoid_controller.get_pose()
        print("BASE POS2:", obj._articulated_agent.base_pos)
        
        arg_name = obj._humanoid_joint_action._action_arg_prefix + "human_joints_trans"
        arg_dict = {arg_name: human_joints_trans}
        obj._humanoid_joint_action.step(**arg_dict)

        print("NEW BASE POS:", obj._articulated_agent.base_pos)
        print("BASE_TRANSFORMATION 2:", obj._articulated_agent.base_transformation)

        base_pos = self.art_agent.base_pos
        self.art_agent.base_pos = base_pos

        print("Final BASE POS:", self.art_agent.base_pos)
        print("BASE_TRANSFORMATION 3:", obj._articulated_agent.base_transformation)


        # new_joints = human_joints_trans[:-32]
        # new_pos_transform_base = human_joints_trans[-16:]
        # new_pos_transform_offset = human_joints_trans[-32:-16]

        # # When the array is all 0, this indicates we are not setting
        # # the human joint
        # if np.array(new_pos_transform_offset).sum() != 0:
            # print('ENtering if BLOCK!!!')
            # vecs_base = [
                # mn.Vector4(new_pos_transform_base[i * 4 : (i + 1) * 4])
                # for i in range(4)
            # ]
            # vecs_offset = [
                # mn.Vector4(new_pos_transform_offset[i * 4 : (i + 1) * 4])
                # for i in range(4)
            # ]
            # new_transform_offset = mn.Matrix4(*vecs_offset)
            # new_transform_base = mn.Matrix4(*vecs_base)
            # if (
                # new_transform_offset.is_rigid_transformation()
                # and new_transform_base.is_rigid_transformation()
            # ):
                # # TODO: this will cause many sampled actions to be invalid
                # # Maybe we should update the sampling mechanism
                # obj._articulated_agent.set_joint_transform(
                    # new_joints, new_transform_offset, new_transform_base
                # )
        # joint_action = obj._humanoid_joint_action
        # # print('action arg prefix!!!:', joint_action._action_arg_prefix)
        # key = joint_action._action_arg_prefix + 'human_joints_trans'
        # # key = f'agent_{obj._agent_id}' + '_human_joint_trans'
        # # print('KEY:', key)
        # arg_dict = dict()
        # arg_dict[key] = new_pose
        # # print('ARG_DICT', arg_dict)
        # joint_action.step(**arg_dict)


        

class OpenGripperAction(Action):
    def applyTo(self, obj, sim):
        obj._articulated_agent.open_gripper()

class CloseGripperAction(Action):
    def applyTo(self, obj, sim):
        obj._articulated_agent.close_gripper()

class SnapToObjectAction(Action):
    def __init__(self, target_obj):
        self.target_obj_id = target_obj._object_id

    def applyTo(self, obj, sim):
        obj._grasp_manager.snap_to_obj((self.target_obj_id))


