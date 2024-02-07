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
import habitat
from habitat_sim.physics import JointMotorSettings, MotionType
from omegaconf import OmegaConf

from scenic.core.simulators import *


class GoRelDeltaAction(Action):

    def __init__(self, dx=0, dy=0, dz=0, rot=0):
        self.pos_delta = mn.Vector3(dx, dy, dz)
        #TODO add rotation delta
    
    def applyTo(self, obj, sim):
        art_agent = sim.sim.articulated_agent
        art_agent.base_pos = art_agent.base_pos + self.pos_delta
        return

