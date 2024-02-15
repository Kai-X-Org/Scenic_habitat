"""Simulator interface for Gazebo."""
import logging
import math
import os
import traceback
import warnings


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
            **kwargs,
        )

    def destroy(self):
        # TODO add code to be run when Scenic runs terminates, if needed
        super().destroy()


class HabitatSimulation(Simulation):
    """
    Simulation class for Gazebo-Scenic
    gazebo_<xyz>_ground_truth: the offset FROM the Gazebo frame TO the robot frame
    gazebo_yaw_ground_truth: true offset FROM the Gazebo frame TO the robot frame"""

    def __init__(self, scene, client, render, record, timestep=0.1, **kwargs):
        print('initializing!')
        self.client = client
        self.render = True
        self.record = record
        self.timestep = timestep
        self.step_actions = []
        if 'cfg' in kwargs:
            self.cfg = kwargs['cfg']
        self.agent_dict = dict()
        self.sim = None
        self.observations = list()
        self.ego = None
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        agent_count = 0
        # print('entering loop!')
        # print(self.scene.objects)
        # print(self.agents)
        for obj in self.scene.objects:
            # print("entered loop!")
            if obj.is_agent:
                # self.ego = obj
                obj._agent_id = agent_count
                agent_count += 1

                sim_sensors = { # TODO temporary
                    "third_rgb": ThirdRGBSensorConfig(),
                    "head_rgb": HeadRGBSensorConfig(),
                }
                x, y, z = obj.position
                agent_config = utils.create_agent_config(obj.name, obj.object_type, obj.urdf_path, 
                                    x, y, z, obj.roll, obj.pitch, obj.yaw, sim_sensors=sim_sensors)
                self.agent_dict['main_agent'] = agent_config # TODO change this for multi agets

            else:
                handle = obj.handle
            # TODO add in the rest!!!
        print(self.agent_dict)
        self.sim = utils.init_rearrange_sim(self.agent_dict)
        # art_agent = self.sim.articulated_agent
        # art_agent.sim_obj.motion_type = MotionType.DYNAMIC
        # art_agent.base_pos = mn.Vector3(self.ego.position[0], 
                                        # self.ego.position[1], self.ego.position[2]) # TODO temporary solution
        super().setup()  # Calls createObjectInSimulator for each object
        self.sim.step({}) # TODO is this needed???
        self.observations.append(self.sim.get_sensor_observations())
        print(self.observations[0].keys())
        # print(art_agent.params.cameras.keys())
        return

    def createObjectInSimulator(self, obj):
        """
        Spawns the object in the Gazebo simulator.
        If the object has a mesh, adds it to the collision_world
        to enable collision avoidance
        Args:
        obj: the scenic object, needs to have a name and position field

        Returns:
        Tuple(bool success, status_message)
        """
        # TODO add in mechanism to handle different types of agent
        # TODO need someway to pass on the agent_id field
        # Proposal, each agent gets a _agent_id field, that is set in setup() above
        if obj.is_agent:
            art_agent = self.sim.agents_mgr[obj._agent_id].articulated_agent # TODO what to do with this line? 
            art_agent.sim_obj.motion_type = MotionType.DYNAMIC # TODO fixe the physics
            art_agent.base_pos = mn.Vector3(obj.position[0], 
                                            obj.position[1], obj.position[2]) # TODO temporary solution
            art_agent.base_rot = obj.yaw # ROTATION IS just the Yaw angle...can also
                                    # set it directly with art_agent.sim_obj.rotation = <Quaternion>

        else:
            handle = obj.handle
            # TODO add in the rest!!!

    def executeActions(self, allActions):
        """
        execute action for each object. Does not immediately render,
        but instead buffers the object
        """
        for agent, actions in allActions.items():
            for action in actions:
                try:
                    a = action.applyTo(agent, self)
                except Exception as e:
                    print(f"Failed to execute action, exception:\n{str(e)}")
                    logging.error(traceback.format_exc())
        return

    def step(self):
        print("stepping!")
        self.sim.step_physics(self.timestep)
        self.observations.append(self.sim.get_sensor_observations())


    def getProperties(self, obj, properties):
        print(self.sim.articulated_agent.base_pos)
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
        # TODO do the rendering here
        # print('destroying!!!')
        # print(self.observations)
        vut.make_video(
            self.observations,
            "scene_camera_rgb",
            "color",
            "/home/ek65/Scenic-habitat/src/scenic/simulators/habitat/robot_tutorial_video",
            open_vid=False,
        )
        vut.make_video(
            self.observations,
            "third_rgb",
            "color",
            "/home/ek65/Scenic-habitat/src/scenic/simulators/habitat/robot_tutorial_video_new_order",
            open_vid=False,
        )
        super().destroy()
        return

    def HabitatToRobotMap(self, pose):
        """
        Converts from the gazebo map frame to the Robot map frame
        Args:
        pose = Tuple(x, y, z, yaw)
        """
        x_offset = self.gazebo_x_ground_truth
        y_offset = self.gazebo_y_ground_truth
        z_offset = self.gazebo_z_ground_truth
        yaw_offset = self.gazebo_yaw_ground_truth
        yaw_rotation = self.gazebo_to_robot_yaw_rot

        g = np.array(
            [
                [np.cos(yaw_offset), -np.sin(yaw_offset), x_offset],
                [np.sin(yaw_offset), np.cos(yaw_offset), y_offset],
                [0, 0, 1],
            ]
        )
        g = np.linalg.inv(g)

        x, y, _ = g @ np.array(list(pose[:2]) + [1])
        z = pose[2] - z_offset
        yaw = pose[3] - yaw_offset

        return (x, y, z, yaw)

    def RobotToGazeboMap(self, pose):
        """
        Converts from the Robot map frame to the gazebo map frame
        Args:
        pose: (x, y, z, yaw)
        """
        x_offset = self.gazebo_x_ground_truth
        y_offset = self.gazebo_y_ground_truth
        z_offset = self.gazebo_z_ground_truth
        yaw_offset = self.gazebo_yaw_ground_truth
        g = np.array(
            [
                [np.cos(yaw_offset), -np.sin(yaw_offset), x_offset],
                [np.sin(yaw_offset), np.cos(yaw_offset), y_offset],
                [0, 0, 1],
            ]
        )
        x, y, _ = g @ np.array(list(pose[:2]) + [1])

        z = pose[2] + z_offset
        yaw = pose[3] + yaw_offset

        return (x, y, z, yaw)

    def ScenicToRobotMap(self, pose, obj=None):
        """
        Converts from the Scenic map coordinate to the Robot map frame
        Args:
        pose: (x, y, z, yaw)
        """
        assert len(pose) == 4
        x, y, z, yaw = pose
        if obj and hasattr(obj, "positionOffset"):
            dx, dy, dz = (
                obj.positionOffset[0],
                obj.positionOffset[1],
                obj.positionOffset[2],
            )
            x = x + dx
            y = y + dy
            z = z + dz

        return (x, y, z, yaw + math.pi / 2)

    def RobotToScenicMap(self, pose, obj=None):
        """
        Converts from the Robot 'map' frame coordinate to the Scenic map coordinate
        Args:
        pose: (x, y, z, yaw)
        """
        assert len(pose) == 4
        x, y, z, yaw = pose

        if obj and hasattr(obj, "positionOffset"):
            dx, dy, dz = (
                obj.positionOffset[0],
                obj.positionOffset[1],
                obj.positionOffset[2],
            )
            x = x - dx
            y = y - dy
            z = z - dz
        return (x, y, z, yaw - math.pi / 2)

    def ScenicToHabitatMap(self, pose, obj=None):
        """
        Converts from the Scenic map coordinate to the Gazebo map frame coordinate
        Args:
        pose: (x, y, z, yaw)
        """
        # assert len(pose) == 4
        return self.RobotToHabitatMap(self.ScenicToRobotMap(pose, obj=obj))

    def HabitatToScenicMap(self, pose, obj=None):
        """
        Converts from the Gazebo map frame coordinate to the Scenic map coordinate
        Args:
        pose: (x, y, z, yaw)
        """
        # assert len(pose) == 4
        return self.RobotToScenicMap(self.HabitatToRobotMap(pose), obj=obj)
