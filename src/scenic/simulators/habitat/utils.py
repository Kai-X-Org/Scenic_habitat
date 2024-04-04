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

    
    cfg = OmegaConf.create(sim_cfg)

    # Set the scene agents
    cfg.agents = agent_dict
    cfg.agents_order = list(cfg.agents.keys())
    return cfg

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
    cfg = OmegaConf.create(sim_cfg)
    
    # Create the scene
    sim = RearrangeSim(cfg)

    # This is needed to initialize the agents
    sim.agents_mgr.on_new_scene()

    # For this tutorial, we will also add an extra camera that will be used for third person recording.
    camera_sensor_spec = habitat_sim.CameraSensorSpec()
    camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    camera_sensor_spec.uuid = "scene_camera_rgb"
    camera_sensor_spec.position = mn.Vector3(0, 4, 7)
    camera_sensor_spec.orientation = mn.Vector3(-1.57, 0, 0)
    camera_sensor_spec.resolution = mn.Vector2i(1024, 1024)

    # TODO: this is a bit dirty but I think its nice as it shows how to modify a camera sensor...
    sim.add_sensor(camera_sensor_spec, 0)

    return sim

def set_agent_state(agent, position, orientation):
    agent_state = habitat_sim.AgentState()
    agent_state.position = position
    agent_state.orientation = orientation
    agent.set_state(agent_state)


def get_agent_state(agent):
    return agent.get_state()

# def make_cfg(settings):
    # sim_cfg = habitat_sim.SimulatorConfiguration()
    # sim_cfg.gpu_device_id = 0
    # sim_cfg.scene_id = settings["scene"]
    # sim_cfg.enable_physics = settings["enable_physics"]

    # # Note: all sensors must have the same resolution
    # sensors = {
        # "color_sensor": {
            # "sensor_type": habitat_sim.SensorType.COLOR,
            # "resolution": [settings["height"], settings["width"]],
            # "position": [0.0, settings["sensor_height"], 0.0],
        # },
        # "depth_sensor": {
            # "sensor_type": habitat_sim.SensorType.DEPTH,
            # "resolution": [settings["height"], settings["width"]],
            # "position": [0.0, settings["sensor_height"], 0.0],
        # },
        # "semantic_sensor": {
            # "sensor_type": habitat_sim.SensorType.SEMANTIC,
            # "resolution": [settings["height"], settings["width"]],
            # "position": [0.0, settings["sensor_height"], 0.0],
        # },
    # }

    # sensor_specs = []
    # for sensor_uuid, sensor_params in sensors.items():
        # if settings[sensor_uuid]:
            # sensor_spec = habitat_sim.SensorSpec()
            # sensor_spec.uuid = sensor_uuid
            # sensor_spec.sensor_type = sensor_params["sensor_type"]
            # sensor_spec.resolution = sensor_params["resolution"]
            # sensor_spec.position = sensor_params["position"]

            # sensor_specs.append(sensor_spec)

    # # Here you can specify the amount of displacement in a forward action and the turn angle
    # agent_cfg = habitat_sim.agent.AgentConfiguration()
    # agent_cfg.sensor_specifications = sensor_specs
    # agent_cfg.action_space = {
        # "move_forward": habitat_sim.agent.ActionSpec(
            # "move_forward", habitat_sim.agent.ActuationSpec(amount=0.25)
        # ),
        # "turn_left": habitat_sim.agent.ActionSpec(
            # "turn_left", habitat_sim.agent.ActuationSpec(amount=30.0)
        # ),
        # "turn_right": habitat_sim.agent.ActionSpec(
            # "turn_right", habitat_sim.agent.ActuationSpec(amount=30.0)
        # ),
    # }

    # return habitat_sim.Configuration(sim_cfg, [agent_cfg])
# COULD BE USEFUL?
def remove_all_objects(sim):
    for id in sim.get_existing_object_ids():
        sim.remove_object(id)
