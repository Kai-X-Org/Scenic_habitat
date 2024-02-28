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
        if obj._articulated_agent_type == 'KinematicHumanoid':
            rel_pose = self.art_agent.base_pos + self.pos_delta
            new_pose = obj._humanoid_controller.calculate_walk_pose(rel_pose)
            joint_action = HumanoidJointAction(new_pose, sim=sim.sim)
        else:
            self.art_agent.base_pos = self.art_agent.base_pos + self.pos_delta
        return

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


