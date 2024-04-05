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
    _spawn: True

    def distanceToClosest(self, type: type) -> Object:
        """Compute the distance to the closest object of the given type.

        For example, one could write :scenic:`self.distanceToClosest(Car)` in a behavior.
        """
        objects = simulation().objects
        minDist = float('inf')
        for obj in objects:
            if not isinstance(obj, type):
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
    _policy_path_dict: dict()
    _object_template_handle: None
    _has_grasp: True
    _grasp_manager: None

    def distanceToClosest(self, type: type) -> Object:
        """Compute the distance to the closest object of the given type.

        For example, one could write :scenic:`self.distanceToClosest(Car)` in a behavior.
        """
        objects = simulation().objects
        minDist = float('inf')
        for obj in objects:
            if not isinstance(obj, type):
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

class SpotRobot(Robot):
    name: "SpotRobot"
    object_type: "SpotRobot"
    _articulated_agent_type: "SpotRobot"
    urdf_path: data_dir + 'robots/hab_spot_arm/urdf/hab_spot_arm.urdf'
    _policy_path_dict: dict(pick='/home/ek65/Scenic-habitat/src/scenic/simulators/habitat/policies/pick_latest.torchscript',
                       place='/home/ek65/Scenic-habitat/src/scenic/simulators/habitat/policies/place_latest_sample.torchscript')
    _policies: dict()
    shape: CylinderShape(dimensions=(0.508,0.559,1.096)) # TODO change this. 

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
    is_agent: False
    _object_id: None
    _use_file_handle: None
    _object_file_handle: None
    _object_template_handle: None
    _managed_rigid_object: None
    _spawn: True # whether this object should be spawned or not

class MasterChef(HabitatObject):
    name: 'MasterChef'
    object_type: 'MasterChef'
    _use_file_handle: True
    _object_file_handle: data_dir + 'objects/ycb/configs/002_master_chef_can.object_config.json'
    shape: CylinderShape(dimensions=(0.1,0.1,0.5)) # TODO just a dummy dimensions

class TennisBall(HabitatObject):
    name: 'TennisBall'
    object_type: 'TennisBall'
    _use_file_handle: True
    _object_file_handle: data_dir + 'objects/ycb/configs/056_tennis_ball.object_config.json'
    shape: CylinderShape(dimensions=(0.1,0.1,0.5)) # TODO just a dummy dimensions

class GelatinBox(HabitatObject):
    name: 'GelatinBox'
    object_type: 'GelatinBox'
    _use_file_handle: True
    _object_file_handle: data_dir + 'objects/ycb/configs/009_gelatin_box.object_config.json'
    shape: BoxShape(dimensions=(0.1,0.1,0.1)) # TODO just a dummy dimensions

class Surface(HabitatObject):
    name: 'Surface'
    _spawn: False


