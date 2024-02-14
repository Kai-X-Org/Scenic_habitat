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



class Robot(HabitatAgent):
    name: 'robot'
    object_type: 'robot'
    urdf_path: ''
    is_agent: True
    position: (0, 0, 0)
    yaw: 0
    roll: 0
    pitch: 0

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
    urdf_path: data_dir + 'robots/hab_fetch/robots/hab_fetch.urdf'
    
    shape: CylinderShape(dimensions=(0.508,0.559,1.096))

