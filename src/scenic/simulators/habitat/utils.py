import habitat_sim
import pdb
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
from habitat.config.default_structured_configs import TaskConfig, EnvironmentConfig, DatasetConfig, HabitatConfig
from habitat.config.default_structured_configs import ArmActionConfig, BaseVelocityActionConfig, OracleNavActionConfig, ActionConfig
from habitat.core.env import Env
from omegaconf import OmegaConf
import os

def make_sim_cfg(agent_dict):
    # Start the scene config
    sim_cfg = SimulatorConfig(type="RearrangeSim-v0") # TODO change this for general sim in the future
    
    data_path = '/home/ek65/habitat-lab/data/'
    # This is for better graphics
    sim_cfg.habitat_sim_v0.enable_hbao = True
    sim_cfg.habitat_sim_v0.enable_physics = True

    
    # Set up an example scene
    sim_cfg.scene = os.path.join(data_path, "hab3_bench_assets/hab3-hssd/scenes/103997919_171031233.scene_instance.json")
    sim_cfg.scene_dataset = os.path.join(data_path, "hab3_bench_assets/hab3-hssd/hab3-hssd.scene_dataset_config.json")
    sim_cfg.additional_object_paths = [os.path.join(data_path, 'objects/ycb/configs/')]
    
    print(sim_cfg.scene)
    print(sim_cfg.scene_dataset)
    
    cfg = OmegaConf.create(sim_cfg)
    print('finished init configs!!!')
    # Set the scene agents
    cfg.agents = agent_dict
    cfg.agents_order = list(cfg.agents.keys())
    return cfg

def make_hab_cfg(agent_dict, action_dict, lab_sensor_dict, timestep=1):
    """
    Make the configurations for habitat env
    """
    sim_cfg = make_sim_cfg(agent_dict)
    task_cfg = TaskConfig(type="RearrangeEmptyTask-v0")
    # task_cfg = TaskConfig()
    task_cfg.actions = action_dict
    task_cfg.lab_sensors = lab_sensor_dict
    env_cfg = EnvironmentConfig()
    # FIXME line below has hardcoded directory
    dataset_cfg = DatasetConfig(type="RearrangeDataset-v0", 
                                data_path="/home/ek65/habitat-lab/data/hab3_bench_assets/episode_datasets/small_large.json.gz") 
    # dataset_cfg = DatasetConfig(type="RearrangeDataset-v0") 
    # dataset_cfg = DatasetConfig() 

    task_cfg.physics_target_sps = 1/timestep # This communicates the Scenic timestep to habitat

    hab_cfg = HabitatConfig()
    hab_cfg.environment = env_cfg
    hab_cfg.task = task_cfg
    hab_cfg.dataset = dataset_cfg
    hab_cfg.simulator = sim_cfg
    hab_cfg.simulator.seed = hab_cfg.seed

    return hab_cfg

def create_agent_config(name, agent_type, urdf_path, motion_data_path=None, sim_sensors=None):
    # TODO add cases for humanoids!!!
    main_agent_config = AgentConfig()
    main_agent_config.articulated_agent_urdf = urdf_path
    main_agent_config.articulated_agent_type = agent_type
    main_agent_config.sim_sensors = sim_sensors
    if motion_data_path:
        main_agent_config.motion_data_path = motion_data_path
    return main_agent_config

def init_rearrange_sim(agent_dict):
    # Start the scene config
    sim_cfg = make_sim_cfg(agent_dict)    
    print('FINISHED MAKING SIM CFG') 
    cfg = OmegaConf.create(sim_cfg)
    
    # Create the scene
    sim = RearrangeSim(cfg)

    # This is needed to initialize the agents
    sim.agents_mgr.on_new_scene()

    # For this tutorial, we will also add an extra camera that will be used for third person recording.
    camera_sensor_spec = habitat_sim.CameraSensorSpec()
    camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    camera_sensor_spec.uuid = "scene_camera_rgb"

    # TODO: this is a bit dirty but I think its nice as it shows how to modify a camera sensor...
    sim.add_sensor(camera_sensor_spec, 0)

    return sim

def init_rearrange_env(agent_dict, action_dict, lab_sensor_dict, timestep=1):
    """
    Initializes the rearrangement environment
    """
    hab_cfg = make_hab_cfg(agent_dict, action_dict, lab_sensor_dict, timestep=timestep)
    res_cfg = OmegaConf.create(hab_cfg)
    breakpoint()
    return Env(res_cfg)

def add_scene_camera(env, name='scene_camera_rgb', camera_pos: mn.Vector3 = mn.Vector3(0, 4, 7),
                     orientation: mn.Vector3 = mn.Vector3(-1.57, 0, 0),
                     resolution: mn.Vector2i = (mn.Vector2i(1024, 1024))):

    camera_sensor_spec = habitat_sim.CameraSensorSpec()
    camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    camera_sensor_spec.uuid = name
    camera_sensor_spec.position = mn.Vector3(0, 4, 7)
    camera_sensor_spec.orientation = mn.Vector3(-1.57, 0, 0)
    camera_sensor_spec.resolution = mn.Vector2i(1024, 1024)
    env.sim.add_sensor(camera_sensor_spec, 0)


def set_agent_state(agent, position, orientation):
    agent_state = habitat_sim.AgentState()
    agent_state.position = position
    agent_state.orientation = orientation
    agent.set_state(agent_state)


def get_agent_state(agent):
    return agent.get_state()

def remove_all_objects(sim):
    for id in sim.get_existing_object_ids():
        sim.remove_object(id)
