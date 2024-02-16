import math
from scenic.core.utils import repairMesh
from scenic.simulators.habitat.simulator import HabitatSimulation, HabitatSimulator
import trimesh


simulator HabitatSimulator()
data_dir = '/home/ek65/habitat-lab/data/'

class HabitatAgent():
    name: 'agent'
    object_type: 'agent'
    is_agent: True
    _agent_id: None
    _articulated_agent_type: None
    _motion_data_path: None
    _articulated_agent: None



class Robot(HabitatAgent):
    name: 'robot'
    object_type: 'robot'
    _articulated_agent_type: None
    urdf_path: ''
    is_agent: True
    position: (0, 0, 0)
    yaw: 0
    roll: 0
    pitch: 0
    _object_template_handle: None

    def distanceToClosest(self, object_class):
        objects = simulation().objects
        minDist = float('inf')
        for obj in objects:
            if not isinstance(obj, object_class):
                continue
            d = distance from self to obj
            if 0 < d < minDist:
                minDist = d
        return minDist

    def getClosest(self, object_class):
        objects = simulation().objects
        minDist = float('inf')
        tgt = None
        for obj in objects:
            if not isinstance(obj, object_class):
                continue
            d = distance from self to obj
            if 0 < d < minDist:
                minDist = d
                tgt = obj
        return tgt

class FetchRobot(Robot):
    name: "FetchRobot"
    object_type: 'FetchRobot'
    _articulated_agent_type: 'FetchRobot'
    urdf_path: data_dir + 'robots/hab_fetch/robots/hab_fetch.urdf'
    
    shape: CylinderShape(dimensions=(0.508,0.559,1.096))

class KinematicHumanoid(HabitatAgent):
    name: "Humanoid"
    object_type: 'KinematicHumanoid'
    _articulated_agent_type: 'KinematicHumanoid'
    _humanoid_controller: None
    urdf_path: None
    shape: CylinderShape(dimensions=(0.508,0.559,1.75))

class Female_0(KinematicHumanoid):
    name: "Female_0"
    urdf_path: data_dir + 'hab3_bench_assets/humanoids/female_0/female_0.urdf'
    _motion_data_path: data_dir + 'hab3_bench_assets/humanoids/female_0/female_0_motion_data_smplx.pkl'


class HabitatObject:
    name: 'HabitatObject'
    object_type: None
    _object_id: None
    _use_file_handle: None
    _object_file_handle: None
    _object_template_handle: None

class MasterChef(HabitatObject)
    name: 'MasterChef'
    object_type: 'MasterChef'
    _use_file_handle: True
    _object_file_handle: data_dir + 'objects/ycb/configs/002_master_chef_can.object_config.json'
    shape: CylinderShape(dimensions=(0.1,0.1,0.5)) # TODO just a dummy dimensions
