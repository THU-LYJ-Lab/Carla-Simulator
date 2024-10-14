import glob
import sys
import os
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from export_utils import *
from export_callbacks import overall_sensor_callback, semseg_callback, insseg_callback, lidar_callback

class SensorManager(object):
        
    def __init__(self, world, car, sensor_queue, vehicle_ids, walker_ids):
        self.surface = None
        self.world = world
        self.car = car

        # Create Cameras
        camera_locations = [
            carla.Location(x=1.5, z=2.4), # front
            carla.Location(x=1.5, y=0.5, z=2.4), # front left
            carla.Location(x=1.5, y=-0.5, z=2.4), # front right
            carla.Location(x=-1.5, z=2.4), # back
            carla.Location(x=-1.5, y=0.5, z=2.4), # back left
            carla.Location(x=-1.5, y=-0.5, z=2.4), # back right
        ]
        camera_rotations = [
            carla.Rotation(), # front
            carla.Rotation(pitch=-15, yaw=-45), # front left
            carla.Rotation(pitch=-15, yaw=45), # front right
            carla.Rotation(yaw=180), # back
            carla.Rotation(pitch=-15, yaw=-135), # back left
            carla.Rotation(pitch=-15, yaw=135), # back right
        ]
        camera_names = [
            "front",
            "front_left",
            "front_right",
            "back",
            "back_left",
            "back_right",
        ]
        callback_funcs = []
        print(vehicle_ids)
        for i in range(6):
            grp_callback_funcs = []
            grp_callback_funcs.append(lambda image, i=i: overall_sensor_callback(image, 'data/images/%s/%.6d.jpg' % (camera_names[i], image.frame), sensor_queue, camera_names[i] + '_rgb'))
            grp_callback_funcs.append(lambda image, i=i: overall_sensor_callback(image, 'data/depths/depth_%s/%.6d.exr' % (camera_names[i], image.frame), sensor_queue,camera_names[i] + '_depth'))
            grp_callback_funcs.append(lambda image, i=i: semseg_callback(image, 'data/semseg/%s' % (camera_names[i]), sensor_queue, camera_names[i] + '_semseg'))
            grp_callback_funcs.append(lambda image, i=i: insseg_callback(image, 'data/insseg/%s' % (camera_names[i]), sensor_queue, camera_names[i] + '_insseg', vehicle_ids, walker_ids))
            callback_funcs.append(grp_callback_funcs)
        
        self.cameras = create_ego_camera_sensors(self.world, 1920, 1080, 110, camera_locations, camera_rotations, self.car.ego_vehicle, callback_funcs)

        # Create Lidars
        lidar_locations = [
            carla.Location(x=0, y=0, z=2), # top
            carla.Location(x=1.5, y=0, z=1.5), # front
            carla.Location(x=-1.5, y=0, z=1.5), # rear
            carla.Location(x=0, y=-1.5, z=1.5), # left
            carla.Location(x=0, y=1.5, z=1.5), # right
        ]
        lidar_rotations = [
            carla.Rotation(), # top
            carla.Rotation(yaw=0), # front
            carla.Rotation(yaw=180), # rear
            carla.Rotation(yaw=-90), # left
            carla.Rotation(yaw=90), # right
        ]
        lidar_names = [
            "top",
            "front",
            "rear",
            "left",
            "right",
        ]
        lidar_callback_funcs = []

        for i in range(5):
            lidar_callback_funcs.append(lambda data, i=i: lidar_callback(data,'data/lidars/lidar_%s' % lidar_names[i], sensor_queue, 'lidar_' + lidar_names[i]))

        self.lidars = create_ego_lidar_set(self.world, 32, 50, 0.1, 10, 100000, lidar_locations, lidar_rotations, self.car.ego_vehicle, lidar_callback_funcs)
