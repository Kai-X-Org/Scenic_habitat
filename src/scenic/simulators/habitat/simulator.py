"""Simulator interface for Meta Habitat."""
import logging
import math
import os
import traceback
import warnings
import torch

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
from habitat.articulated_agent_controllers import HumanoidRearrangeController, HumanoidSeqPoseController
from omegaconf import OmegaConf
from habitat.config.default_structured_configs import HumanoidJointActionConfig, HumanoidPickActionConfig
from habitat.tasks.rearrange.actions.actions import HumanoidJointAction

import scenic.core.errors as errors
from scenic.core.simulators import Simulation, SimulationCreationError, Simulator
from scenic.core.vectors import Vector
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
import scenic.simulators.habitat.utils as utils

if errors.verbosityLevel == 0:  # suppress pygame advertisement at zero verbosity
    os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
import pygame

from scenic.core.simulators import SimulationCreationError
from scenic.syntax.veneer import verbosePrint

# TODO: Import Robot-specific library


class HabitatSimCreationError(SimulationCreationError):
    """
    If anything went wrong in setting up the scene, this error is thrown.
    If Scenic is run using the CLI, The current scene terminates, and a new scene is started
    Args:
    String msg: the message to be given
    """

    def __init__(self, msg):
        self.msg = msg
        super().__init__(self.msg)


class HabitatSimRuntimeError(SimulationCreationError):
    """
    If anything went wrong in running the scene, this error is thrown.
    If Scenic is run using the CLI, The current scene terminates, and a new scene is started
    Args:
    String msg: the message to be given
    Exception exc: the exception thrown by other parts of the code that makes us stop scene
    """

    def __init__(self, msg, exc):
        self.msg = exc.args[0]
        self.file_name, self.lineno = exc.filename, exc.lineno
        super().__init__(self.msg)
        self.with_traceback(exc.__traceback__)


class HabitatSimulator(Simulator):
    """Implementation of `Simulator`."""

    def __init__(
        self,
        map_path="",
        timeout=10,
        render=True,
        record="",
        timestep=0.1,
    ):
        super().__init__()
        verbosePrint(f"Connecting to Habitat simulator")
        self.timestep = timestep
        self.render = (
            render  # visualization mode ON/OFF, for future use with fast physics
        )
        self.record = record
        self.scenario_number = 0

        # TODO Decide the form of your client
        self.client = dict()

    def createSimulation(self, scene, timestep, **kwargs):
        if timestep is not None and timestep != self.timestep:
            raise RuntimeError(
                "cannot customize timestep for individual Habitat simulations; "
                "set timestep when creating the HabitatSimulator instead"
            )
        self.scenario_number += 1

        return HabitatSimulation(
            scene,
            self.client,
            self.render,
            self.record,
            timestep=self.timestep,
            scenario_number=self.scenario_number,
            **kwargs,
        )

    def destroy(self):
        # TODO add code to be run when Scenic runs terminates, if needed
        super().destroy()


class HabitatSimulation(Simulation):
    """
    Simulation class for Habitat-Scenic
    """

    def __init__(self, scene, client, render, record, timestep=0.1, scenario_number=0, **kwargs):
        print('initializing!')
        self.client = client
        self.render = True
        self.record = record
        self.timestep = timestep
        if 'cfg' in kwargs:
            self.cfg = kwargs['cfg']
        self.agent_dict = dict()
        self.sim = None
        self.observations = list()
        self.ego = None
        self.habitat_agents = list()
        self.scenario_number = scenario_number  # used for naming of videos
        self.device = torch.device('cuda') # I think this is right?\
        self.step_action_dict = dict()
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        agent_count = 0
        agent_names = []
        
        # getting the agent configs
        action_dict = dict()
        for obj in self.scene.objects:
            # print("entered loop!")
            if obj.is_agent:
                # self.ego = obj
                self.habitat_agents.append(obj)
                obj._agent_id = agent_count
                agent_count += 1

                sim_sensors = { # TODO temporary
                    "third_rgb": ThirdRGBSensorConfig(),
                    "head_rgb": HeadRGBSensorConfig(),
                }
                x, y, z = obj.position
                agent_config = utils.create_agent_config(obj.name, obj._articulated_agent_type, obj.urdf_path, 
                                    motion_data_path=obj._motion_data_path, sim_sensors=sim_sensors)

                if obj.name in agent_names:
                    raise HabitatSimCreationError(f"Error: two agents have the same name: {obj.name}")
                else:
                    self.agent_dict[obj.name] = agent_config 

                action_dict.update(obj._action_dict)
        
        print(f"Current Action Dict: {action_dict}")
        self.env = utils.init_rearrange_env(self.agent_dict, action_dict, timestep=self.timestep) 
        print("FINISHED INIT ENV!!!")
        self.sim = self.env.sim
        self.env.reset()
        # self.sim = utils.init_rearrange_sim(self.agent_dict) # DO this if we want to use habitat_sim only

        self.obj_attr_mgr = self.sim.get_object_template_manager()
        self.prim_attr_mgr = self.sim.get_asset_template_manager()
        self.stage_attr_mgr = self.sim.get_stage_template_manager()
        self.rigid_obj_mgr = self.sim.get_rigid_object_manager()

        obs = self.env.step({"action": (), "action_args": {}})
        super().setup()  # Calls createObjectInSimulator for each object
        self.sim.step({}) # TODO is this needed???
        # FIXME remove this now that we are on ENV???
        self.observations.append(self.sim.get_sensor_observations())
        print(self.observations[0].keys())
        return

    def createObjectInSimulator(self, obj):
        """
        Spawns the object in the habitat simulator.
        If the object has a mesh, adds it to the collision_world
        to enable collision avoidance
        Args:
        obj: the scenic object, needs to have a name and position field

        Returns:
        Tuple(bool success, status_message)
        """
        print(f"CREATING {obj.name}")
        for action_name, action_space in self.env.action_space.items():
            print(action_name, action_space)
        if obj.is_agent:
            art_agent = self.env.sim.agents_mgr[obj._agent_id].articulated_agent # TODO what to do with this line? 

            print(f"art_agent: {art_agent}") 
            print(f"art_agent base_pos: {art_agent.base_pos}") 

            obj._articulated_agent = art_agent
            if obj._articulated_agent_type == 'KinematicHumanoid':
                print('data_path:!!!', obj._motion_data_path)
                art_agent.sim_obj.motion_type = MotionType.KINEMATIC # TODO fixe the physics
                art_agent._fixed_base = True  # TODO should this be added?
                obj._humanoid_controller = HumanoidRearrangeController(obj._motion_data_path)
                # obj._humanoid_controller.reset(art_agent.base_transformation)
                obj._humanoid_joint_action = HumanoidJointAction(config=HumanoidJointActionConfig(),
                                                                 sim=self.sim, name=f'agent_{obj._agent_id}')
            else:
                # art_agent.sim_obj.motion_type = MotionType.DYNAMIC # TODO fixe the physics
                art_agent._fixed_base = False
                if obj._has_grasp:
                    obj._grasp_manager = self.sim.agents_mgr[obj._agent_id].grasp_mgrs[0]
                
                extra_files = {"net_meta_dict.pkl": ""} # TODO temporary hardcoding
                for action, model_dir in obj._policy_path_dict.items():
                    obj._policies[action] = torch.jit.load(model_dir, _extra_files=extra_files, map_location=self.device)

            x, y, z, _, _, _ = self.scenicToHabitatMap((obj.position[0], obj.position[1], obj.position[2],0, 0, 0))
            art_agent.base_pos = mn.Vector3(x, y, z) # TODO temporary solution
            art_agent.base_rot = obj.yaw # ROTATION IS just the Yaw angle; can also
            # set it directly with art_agent.sim_obj.rotation = <Quaternion>

        else:
            handle = obj._object_file_handle
            self.obj_attr_mgr.load_configs('/home/ek65/habitat-lab/data/objects/ycb/configs/')
            obj_template_handle = self.obj_attr_mgr.get_template_handles(handle)[0]
            obj._managed_rigid_object = self.rigid_obj_mgr.add_object_by_template_handle(obj_template_handle)

            x, y, z, _, _, _ = self.scenicToHabitatMap((obj.position[0], obj.position[1], obj.position[2],0, 0, 0))
            obj._managed_rigid_object.translation = np.array([x, y, z])
            obj._managed_rigid_object.rotation = mn.Quaternion.rotation(mn.Deg(0), [-1.0, 0.0, 0.0]) # TODO temporary solution
            
            # obj._object_id = self.sim.add_object_by_handle(handle)
            # self.sim.set_translation
            # self.sim.rigid_obj_mgr
            # TODO add in the rest!!!

    def executeActions(self, allActions):
        """
        execute action for each object. Does not immediately render,
        but instead buffers the object
        """

        # TODO things might be different here with the use of Env
        # not really, we could return the empty function
        # or just have the function append to the action dict
        for agent, actions in allActions.items():
            for action in actions:
                try:
                    a = action.applyTo(agent, self)
                except Exception as e:
                    print(f"Failed to execute action, exception:\n{str(e)}")
                    logging.error(traceback.format_exc())
        return

    def step(self):
        # These are for when we are using purely habitat sim
        # self.sim.step_physics(self.timestep)
        # self.observations.append(self.sim.get_sensor_observations())
        self.observations.append(env.step(self.step_action_dict))


    def getProperties(self, obj, properties):
        # print(self.sim.articulated_agent.base_pos)
        if obj.is_agent:
            # agent_state = self.sim.agents_mgr[obj._agent_id].get_state()
            x, y, z = obj._articulated_agent.base_pos
            x, y, z, _, _, _ = self.habitatToScenicMap((x, y, z, 0, 0, 0))
            rotation = obj._articulated_agent.base_rot
            
            d = dict(
                    position=Vector(x, y, z),
                    yaw=rotation,
                    pitch=0,
                    roll=0,
                    speed=0,
                    velocity=Vector(0, 0, 0),
                    angularSpeed=0,
                    angularVelocity=Vector(0, 0, 0),
            )
        else:
            # still need to get the object informations!!!
            d = dict(
                    position=Vector(0, 0, 0),
                    yaw=0,
                    pitch=0,
                    roll=0,
                    speed=0,
                    velocity=Vector(0, 0, 0),
                    angularSpeed=0,
                    angularVelocity=Vector(0, 0, 0),
            )
        return d

    def destroy(self):
        vut.make_video(
            self.observations,
            "scene_camera_rgb",
            "color",
            "/home/ek65/Scenic-habitat/src/scenic/simulators/habitat/robot_tutorial_video",
            open_vid=False,
        )
        # vut.make_video(
            # self.observations,
            # self.habitat_agents[0].name + "_third_rgb",
            # "color",
            # "/home/ek65/Scenic-habitat/src/scenic/simulators/habitat/demo_vid",
            # open_vid=False,
        # )
        vut.make_video(
            self.observations,
            "third_rgb",
            "color",
            "/home/ek65/Scenic-habitat/src/scenic/simulators/habitat/test_spot",
            open_vid=False,
        )
        super().destroy()
        return

    def habitatToRobotMap(self, pose):
        """
        Converts from the habitat map frame to the Robot map frame
        Args:
        pose = Tuple(x, y, z, yaw)
        """
        pass

    def robotToHabitatMap(self, pose):
        """
        Converts from the Robot map frame to the habitat map frame
        Args:
        pose: (x, y, z, yaw)
        """
        pass

    def scenicToRobotMap(self, pose, obj=None):
        """
        Converts from the Scenic map coordinate to the Robot map frame
        Args:
        pose: (x, y, z, yaw)
        """
        pass


    def robotToScenicMap(self, pose, obj=None):
        """
        Converts from the Robot 'map' frame coordinate to the Scenic map coordinate
        Args:
        pose: (x, y, z, yaw)
        """
        assert len(pose) == 4
        pass

    def scenicToHabitatMap(self, pose, obj=None):
        """
        Converts from the Scenic map coordinate to the habitat map frame coordinate
        Args:
        """
        # assert len(pose) == 4
        # return self.RobotToHabitatMap(self.ScenicToRobotMap(pose, obj=obj))
        # g = mn.Matrix4.from_()
        g = np.array([[0, 1, 0, 0], 
                      [0, 0, 1, 0], 
                      [1, 0, 0, 0], 
                      [0, 0, 0, 1]])
        x, y, z, roll, pitch, yaw = pose
        x, y, z, _ = g @ np.array([x, y, z, 1])

        new_roll = pitch
        new_pitch = yaw
        new_yaw = roll
        return (x, y, z, new_roll, new_pitch, new_yaw)



    def habitatToScenicMap(self, pose, obj=None):
        """
        Converts from the habitat map frame coordinate to the Scenic map coordinate
        Args:
        pose: (x, y, z, yaw)
        """
        # assert len(pose) == 4
        # return self.RobotToScenicMap(self.HabitatToRobotMap(pose), obj=obj)
        g = np.array([[0, 1, 0, 0], 
                      [0, 0, 1, 0], 
                      [1, 0, 0, 0], 
                      [0, 0, 0, 1]])
        g = np.linalg.inv(g)
        x, y, z, roll, pitch, yaw = pose
        x, y, z, _ = g @ np.array([x, y, z, 1])
        new_roll = yaw
        new_pitch = roll
        new_yaw = pitch
        return (x, y, z, new_roll, new_pitch, new_yaw)
