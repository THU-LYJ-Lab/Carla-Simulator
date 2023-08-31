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
import open3d as o3d
import numpy as np
import threading
from numpy import random
from queue import Queue, Empty


def overall_sensor_callback(sensor_data, disk_path, sensor_queue, sensor_name):
    # if sensor_data.frame % 10 == 11:
    sensor_data.save_to_disk(disk_path)
    sensor_queue.put(sensor_name)

sensor_queue = Queue()

class SensorManager(object):
    def __init__(self, world, car):
        self.surface = None
        self.world = world
        self.car = car

        #add camera_front
        blueprint_library = world.get_blueprint_library()
        self.camera_bp = blueprint_library.find('sensor.camera.rgb')
        self.camera_bp.set_attribute('image_size_x', '1920')
        self.camera_bp.set_attribute('image_size_y', '1080')
        self.camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera_front = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_front.listen(lambda image: overall_sensor_callback(image, 'tutorial/front/%.6d.jpg' % image.frame, sensor_queue, 'front camera'))

        # add camera_front_left
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, y=0.5,z=2.4),carla.Rotation(pitch=-15, yaw=-45))
        self.camera_front_left = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_front_left.listen(lambda image: overall_sensor_callback(image, 'tutorial/front_left/%.6d.jpg' % image.frame, sensor_queue, 'front left camera'))
        
        #add camera_front_right
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, y=-0.5,z=2.4),carla.Rotation(pitch=-15, yaw=45))
        self.camera_front_right = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_front_right.listen(lambda image: overall_sensor_callback(image, 'tutorial/front_right/%.6d.jpg' % image.frame, sensor_queue, 'front right camera'))
        
        
        #add camera_back
        blueprint_library = world.get_blueprint_library()
        self.camera_bp = blueprint_library.find('sensor.camera.rgb')
        self.camera_bp.set_attribute('image_size_x', '1920')
        self.camera_bp.set_attribute('image_size_y', '1080')
        self.camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=-1.5, z=2.4),carla.Rotation(yaw=180))
        self.camera_back = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_back.listen(lambda image: overall_sensor_callback(image, 'tutorial/back/%.6d.jpg' % image.frame, sensor_queue, 'back camera'))

        # add camera_back_left
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=-1.5, y=0.5,z=2.4),carla.Rotation(pitch=-15, yaw=-135))
        self.camera_back_left = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_back_left.listen(lambda image: overall_sensor_callback(image, 'tutorial/back_left/%.6d.jpg' % image.frame, sensor_queue, 'back left camera'))
        
        #add camera_back_right
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=-1.5, y=-0.5,z=2.4),carla.Rotation(pitch=-15, yaw=135))
        self.camera_back_right = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_back_right.listen(lambda image: overall_sensor_callback(image, 'tutorial/back_right/%.6d.jpg' % image.frame, sensor_queue, 'back right camera'))

        # add depth_front
        blueprint_library = world.get_blueprint_library()
        self.depth_bp = blueprint_library.find('sensor.camera.depth')
        self.depth_bp.set_attribute('image_size_x', '1920')
        self.depth_bp.set_attribute('image_size_y', '1080')
        self.depth_bp.set_attribute('fov', '110')
        depth_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.depth_front = self.world.spawn_actor(self.depth_bp, depth_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.depth_front.listen(
            lambda image: overall_sensor_callback(image, 'tutorial/depth_front/%.6d.exr' % image.frame, sensor_queue,
                                                  'front depth camera'))

        # add depth_front_left
        blueprint_library = world.get_blueprint_library()
        depth_bp = blueprint_library.find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', '1920')
        depth_bp.set_attribute('image_size_y', '1080')
        depth_bp.set_attribute('fov', '110')
        depth_transform = carla.Transform(carla.Location(x=1.5, y=0.5, z=2.4), carla.Rotation(pitch=-15, yaw=-45))
        self.depth_front_left = self.world.spawn_actor(self.depth_bp, depth_transform,
                                                        attach_to=self.car.ego_vehicle)
        # set the callback function
        self.depth_front_left.listen(
            lambda image: overall_sensor_callback(image, 'tutorial/depth_front_left/%.6d.exr' % image.frame, sensor_queue,
                                                  'front left depth camera'))

        # add depth_front_right
        blueprint_library = world.get_blueprint_library()
        depth_bp = blueprint_library.find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', '1920')
        depth_bp.set_attribute('image_size_y', '1080')
        depth_bp.set_attribute('fov', '110')
        depth_transform = carla.Transform(carla.Location(x=1.5, y=-0.5, z=2.4), carla.Rotation(pitch=-15, yaw=45))
        self.depth_front_right = self.world.spawn_actor(self.depth_bp, depth_transform,
                                                         attach_to=self.car.ego_vehicle)
        # set the callback function
        self.depth_front_right.listen(
            lambda image: overall_sensor_callback(image, 'tutorial/depth_front_right/%.6d.exr' % image.frame, sensor_queue,
                                                  'front right depth camera'))

        # add depth_back
        blueprint_library = world.get_blueprint_library()
        self.depth_bp = blueprint_library.find('sensor.camera.depth')
        self.depth_bp.set_attribute('image_size_x', '1920')
        self.depth_bp.set_attribute('image_size_y', '1080')
        self.depth_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180))
        self.depth_back = self.world.spawn_actor(self.depth_bp, depth_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.depth_back.listen(
            lambda image: overall_sensor_callback(image, 'tutorial/depth_back/%.6d.exr' % image.frame, sensor_queue,
                                                  'back depth camera'))

        # add depth_back_left
        blueprint_library = world.get_blueprint_library()
        depth_bp = blueprint_library.find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', '1920')
        depth_bp.set_attribute('image_size_y', '1080')
        depth_bp.set_attribute('fov', '110')
        depth_transform = carla.Transform(carla.Location(x=-1.5, y=0.5, z=2.4), carla.Rotation(pitch=-15, yaw=-135))
        self.depth_back_left = self.world.spawn_actor(self.depth_bp, depth_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.depth_back_left.listen(
            lambda image: overall_sensor_callback(image, 'tutorial/depth_back_left/%.6d.exr' % image.frame, sensor_queue,
                                                  'back left depth camera'))

        # add depth_back_right
        blueprint_library = world.get_blueprint_library()
        depth_bp = blueprint_library.find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', '1920')
        depth_bp.set_attribute('image_size_y', '1080')
        depth_bp.set_attribute('fov', '110')
        depth_transform = carla.Transform(carla.Location(x=-1.5, y=-0.5, z=2.4), carla.Rotation(pitch=-15, yaw=135))
        self.depth_back_right = self.world.spawn_actor(self.depth_bp, depth_transform,
                                                        attach_to=self.car.ego_vehicle)
        # set the callback function
        self.depth_back_right.listen(
            lambda image: overall_sensor_callback(image, 'tutorial/depth_back_right/%.6d.exr' % image.frame, sensor_queue,
                                                  'back right depth camera'))

	    #add lidar_top
        blueprint_library = world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50')
        lidar_transform = carla.Transform(carla.Location(x=0, y=0,z=2))
        self.lidar_top = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.lidar_top.listen(lambda point_cloud: overall_sensor_callback(point_cloud, 'tutorial/lidar_top/%.6d.ply' % point_cloud.frame, sensor_queue, 'top lidar'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def _parse_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

class CarManager(object):
    def __init__(self, world, traffic_manager):
        self.world = world
        self.ego_vehicle = None
        blueprint_library = world.get_blueprint_library()
        self.ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        self.ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # transform = random.choice(world.get_map().get_spawn_points())
        transform = carla.Transform(carla.Location(x = 59.028812408447266, y = 134.06109619140625, z = -0.003951301332563162), carla.Rotation(yaw=179.9717559814453))
        self.ego_vehicle = self.world.spawn_actor(self.ego_vehicle_bp, transform)
        self.ego_vehicle.set_autopilot(True, traffic_manager.get_port())
        world.tick()


def main():
    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(20.0)
        world = client.get_world()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_synchronous_mode(True)
        
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1  # 设置固定帧率
        world.apply_settings(settings)

        car = CarManager(world, traffic_manager)
        sensor = SensorManager(world, car)
        
        frame_count=1000
        f_id = 0
        for _ in range(frame_count):
            print("frame #%d" % f_id)
            f_id += 1
            world.tick()
            try:
                for i in range(13):
                    s_name = sensor_queue.get(True, 2.0)
                    print("receiving data from %s" % s_name)
            except Empty:
                print("some sensor data are missing!")

    finally:
        sensors = world.get_actors().filter('sensor.*')
        for sensor in sensors:
            sensor.close()
            sensor.destroy()
        car.ego_vehicle.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        
         
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user -')
