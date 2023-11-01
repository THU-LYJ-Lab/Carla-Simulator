'''
Usage: lidar2ply input.npz output.ply
This will convert lidar npz in our dataset to a ply file.
'''

import os
import numpy as np
from numpy.matlib import repmat
import sys

args = list(sys.argv)

lidar_file = np.load(args[1])

point_cloud = lidar_file['rays_o'] + lidar_file['rays_d'] * lidar_file['ranges'].reshape((-1, 1))

from plyfile import PlyData, PlyElement

vertices = np.array([(x, y, z) for x, y, z in point_cloud.reshape(-1, 3)], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
plydata = PlyData([PlyElement.describe(vertices, 'vertex')], text=False)
plydata.write(args[2])