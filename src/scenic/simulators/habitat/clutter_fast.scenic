import magnum as mn
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
import math
import time
import numpy as np

import numpy as np
from scenic.core.distributions import distributionFunction, toDistribution
import trimesh

class Table:
    width: 10
    length: 10
    height: 4
    # shape: MeshShape.fromFile(localPath("assets/meshes/coffee_table.obj.bz2"))
    color: [0.404, 0.278, 0.212]

def surface_helper(mesh, dimension, positive, threshold):
    # Drop all faces whose normal vector do not have a sufficiently
    # large component.
    face_normal_vals = mesh.face_normals[:, dimension]
    if positive:
        face_mask = face_normal_vals >= threshold
    else:
        face_mask = face_normal_vals <= threshold

    mesh.faces = mesh.faces[face_mask]
    mesh.remove_unreferenced_vertices()

    # Check if the resulting surface is empty and return an appropriate region.
    if not mesh.is_empty:
        return mesh
    else:
        raise RejectionException("SURFACE_HELPER")

@distributionFunction
def clutter_helper(table, clutter_list, pos, shape, dims, yaw, cm, num_bot_samples=10):
    for clutter in clutter_list:
        cm.add_object("clutter", clutter.occupiedSpace.mesh)

    mesh = MeshVolumeRegion(mesh=shape.mesh, dimensions=dims, position=pos, rotation=Orientation.fromEuler(yaw, 0, 0)).mesh
    # mesh = shape.mesh
    z_shift = 0

    while True:
        min_dist = cm.min_distance_single(mesh)

        if min_dist < 0.01:
            break

        z_shift += min_dist
        mesh.vertices += [0,0,-min_dist]
    
    new_position = Vector(pos[0], pos[1], pos[2]-z_shift+0.001)

    return (new_position, shape, dims, yaw), cm


@distributionFunction
def newCm(foo):
    cm = trimesh.collision.CollisionManager()
    cm.add_object("bed", bed.occupiedSpace.mesh)
    return cm

@distributionFunction
def updateCm(cm, mesh):
    cm.add_object("foo", mesh)
    return cm

@distributionFunction
def toVector(foo) -> Vector:
    return foo



bed_center = Vector(0.5, -6.0, 0.4)
# bed = RectangularRegion(bed_center, 1.57, 2.0, 2.0)

# bed = new Object at bed_center, with yaw 90 deg, with
bed = new Surface at bed_center, with yaw 90 deg, with length 2.0, with width 2.0, with height 1e-8

sampling_height = 1.5
sampling_radius = 0.7

sample_space = RectangularRegion(bed_center + Vector(0, 0, sampling_height), 1.57, sampling_radius, sampling_radius)
clutter_list = []

# shape_distribution = Uniform(BoxShape(), CylinderShape())
# dims_distribution = (Range(1, 2), Range(1, 2), Range(0.15, 0.4))
# yaw_distribution = Range(0, 360 deg)
shape_distribution = Uniform(BoxShape(dimensions=(0.1, 0.1, 0.1)))
dims_distribution = (0.1, 0.1, 0.1)
yaw_distribution = Range(0, 360 deg)

cm = newCm(Range(0,1))

ego = new SpotRobot at (-1.5, -5.5, 0), with yaw -35 deg

for _ in range(20):
    out = clutter_helper(bed, toDistribution(tuple(clutter_list)), 
        (new Point in sample_space).position, 
        resample(shape_distribution),
        dims_distribution,
        resample(yaw_distribution), cm)

    new_obj_info = out[0]
    cm = out[1]

    new_clutter = new GelatinBox at toVector(new_obj_info[0]), with yaw new_obj_info[3]

    clutter_list.append(new_clutter)

    cm = updateCm(cm, new_clutter.occupiedSpace.mesh)

    out = clutter_helper(bed, toDistribution(tuple(clutter_list)), 
        (new Point in sample_space).position, 
        resample(shape_distribution),
        dims_distribution,
        resample(yaw_distribution), cm)

    new_obj_info = out[0]
    cm = out[1]

    new_clutter = new MasterChef at toVector(new_obj_info[0]), with yaw new_obj_info[3]

    clutter_list.append(new_clutter)

    cm = updateCm(cm, new_clutter.occupiedSpace.mesh)

