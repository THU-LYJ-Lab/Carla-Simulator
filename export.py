import glob
import sys
import os
from queue import Queue, Empty
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

def overall_sensor_callback(sensor_data, disk_path, sensor_queue, sensor_name):
    # if sensor_data.frame % 10 == 11:
    sensor_data.save_to_disk(disk_path)
    sensor_queue.put(sensor_name)

def semseg_callback(sensor_data, disk_path, sensor_queue, sensor_name):
    sensor_data.save_to_disk(disk_path, carla.ColorConverter.CityScapesPalette)
    sensor_queue.put(sensor_name)

sensor_queue = Queue()


def get_camera_matrix(transform):
    rotation_matrix = transform.get_matrix()[:3, :3]
    translation_vector = transform.location
    camera_matrix = np.hstack((rotation_matrix, np.array(translation_vector).reshape(3, 1)))
    return camera_matrix


def rotation_matrix_from_euler(rotation: carla.Rotation):
    """
    Convert a CARLA rotation to a 3x3 rotation matrix.
    """
    # Convert degrees to radians
    return np.array(carla.Transform(carla.Location(0, 0, 0), rotation).get_matrix())[:3, :3]

def lidar_callback(data,save_folder,sensor_queue, sensor_name):
    sensor_queue.put(sensor_name)
    # 从原始数据中提取LiDAR数据
    lidar_data = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    lidar_data = np.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))

    # rays_o: LiDAR传感器的位置，+每个数据点重复
    rays_o = np.array([data.transform.location.x, 
                       data.transform.location.y, 
                       data.transform.location.z])
    rays_o = np.tile(rays_o, (lidar_data.shape[0], 1))

    t = carla.Transform(carla.Location(0, 0, 0), data.transform.rotation)

    R = np.array(t.get_matrix())[:3, :3]

    # rays_d: LiDAR数据点减去传感器的位置
    rays_d = np.transpose(R @ np.transpose(lidar_data[:, :3]))

    # 归一化rays_d向量
    norms = np.linalg.norm(rays_d, axis=1)
    rays_d = rays_d / norms[:, np.newaxis]

    # ranges: 到测量值的距离
    ranges = np.linalg.norm(np.transpose(R @ np.transpose(lidar_data[:, :3])), axis=1)

    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
    # 以压缩的npz格式保存数据
    filename = os.path.join(save_folder, f'lidar_frame_{data.frame}.npz')
    np.savez_compressed(filename, rays_o=rays_o, rays_d=rays_d, ranges=ranges)
    
    
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
        self.camera_front.listen(lambda image: overall_sensor_callback(image, 'data/images/front/%.6d.jpg' % image.frame, sensor_queue, 'front camera'))
        #self.camera_front.listen(lambda image: print())
        

        # add camera_front_left
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, y=0.5,z=2.4),carla.Rotation(pitch=-15, yaw=-45))
        self.camera_front_left = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_front_left.listen(lambda image: overall_sensor_callback(image, 'data/images/front_left/%.6d.jpg' % image.frame, sensor_queue, 'front left camera'))
        
        #add camera_front_right
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=1.5, y=-0.5,z=2.4),carla.Rotation(pitch=-15, yaw=45))
        self.camera_front_right = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_front_right.listen(lambda image: overall_sensor_callback(image, 'data/images/front_right/%.6d.jpg' % image.frame, sensor_queue, 'front right camera'))
        
        
        #add camera_back
        blueprint_library = world.get_blueprint_library()
        self.camera_bp = blueprint_library.find('sensor.camera.rgb')
        self.camera_bp.set_attribute('image_size_x', '1920')
        self.camera_bp.set_attribute('image_size_y', '1080')
        self.camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=-1.5, z=2.4),carla.Rotation(yaw=180))
        self.camera_back = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_back.listen(lambda image: overall_sensor_callback(image, 'data/images/back/%.6d.jpg' % image.frame, sensor_queue, 'back camera'))

        # add camera_back_left
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=-1.5, y=0.5,z=2.4),carla.Rotation(pitch=-15, yaw=-135))
        self.camera_back_left = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_back_left.listen(lambda image: overall_sensor_callback(image, 'data/images/back_left/%.6d.jpg' % image.frame, sensor_queue, 'back left camera'))
        
        #add camera_back_right
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=-1.5, y=-0.5,z=2.4),carla.Rotation(pitch=-15, yaw=135))
        self.camera_back_right = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.camera_back_right.listen(lambda image: overall_sensor_callback(image, 'data/images/back_right/%.6d.jpg' % image.frame, sensor_queue, 'back right camera'))

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
            lambda image: overall_sensor_callback(image, 'data/depths/depth_front/%.6d.exr' % image.frame, sensor_queue,
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
            lambda image: overall_sensor_callback(image, 'data/depths/depth_front_left/%.6d.exr' % image.frame, sensor_queue,
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
            lambda image: overall_sensor_callback(image, 'data/depths/depth_front_right/%.6d.exr' % image.frame, sensor_queue,
                                                  'front right depth camera'))

        # add depth_back
        blueprint_library = world.get_blueprint_library()
        self.depth_bp = blueprint_library.find('sensor.camera.depth')
        self.depth_bp.set_attribute('image_size_x', '1920')
        self.depth_bp.set_attribute('image_size_y', '1080')
        self.depth_bp.set_attribute('fov', '110')
        depth_transform = carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180))
        self.depth_back = self.world.spawn_actor(self.depth_bp, depth_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.depth_back.listen(
            lambda image: overall_sensor_callback(image, 'data/depths/depth_back/%.6d.exr' % image.frame, sensor_queue,
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
            lambda image: overall_sensor_callback(image, 'data/depths/depth_back_left/%.6d.exr' % image.frame, sensor_queue,
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
            lambda image: overall_sensor_callback(image, 'data/depths/depth_back_right/%.6d.exr' % image.frame, sensor_queue,
                                                  'back right depth camera'))

	    #add lidar_top
        blueprint_library = world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '100')
        lidar_bp.set_attribute('noise_stddev', '0.1')
        lidar_transform = carla.Transform(carla.Location(x=0, y=0,z=2))
        self.lidar_top = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.car.ego_vehicle)

        # set the callback function
        #self.lidar_top.listen(lambda point_cloud: overall_sensor_callback(point_cloud, 'data/lidars/lidar_top/%.6d.ply' % point_cloud.frame, sensor_queue, 'top lidar'))
        self.lidar_top.listen(lambda data: lidar_callback(data,'data/lidars/lidar_top',sensor_queue, 'top lidar'))


        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50') 
        lidar_bp.set_attribute('noise_stddev', '0.1')
        lidar_bp.set_attribute('rotation_frequency', '10')
        lidar_bp.set_attribute('points_per_second', '100000')

     
        front_lidar_transform = carla.Transform(carla.Location(x=1.5, y=0, z=1.5), carla.Rotation(pitch=0))
        self.front_lidar = self.world.spawn_actor(lidar_bp, front_lidar_transform, attach_to=self.car.ego_vehicle)
        #self.front_lidar.listen(lambda point_cloud: overall_sensor_callback(point_cloud, 'data/lidars/lidar_front/%.6d.ply' % point_cloud.frame, sensor_queue, 'front lidar'))
        self.front_lidar.listen(lambda data: lidar_callback(data,'data/lidars/lidar_front',sensor_queue, 'front lidar'))

        rear_lidar_transform = carla.Transform(carla.Location(x=-1.5, y=0, z=1.5), carla.Rotation(pitch=0, yaw=180))
        self.rear_lidar = self.world.spawn_actor(lidar_bp, rear_lidar_transform, attach_to=self.car.ego_vehicle)
        #self.rear_lidar.listen(lambda point_cloud: overall_sensor_callback(point_cloud, 'data/lidars/lidar_rear/%.6d.ply' % point_cloud.frame, sensor_queue, 'rear lidar'))
        self.rear_lidar.listen(lambda data: lidar_callback(data,'data/lidars/lidar_rear',sensor_queue, 'rear lidar'))


        left_lidar_transform = carla.Transform(carla.Location(x=0, y=-1.5, z=1.5), carla.Rotation(pitch=0, yaw=-90))
        self.left_lidar = self.world.spawn_actor(lidar_bp, left_lidar_transform, attach_to=self.car.ego_vehicle)
        #self.left_lidar.listen(lambda point_cloud: overall_sensor_callback(point_cloud, 'data/lidars/lidar_left/%.6d.ply' % point_cloud.frame, sensor_queue, 'left lidar'))
        self.left_lidar.listen(lambda data: lidar_callback(data,'data/lidars/lidar_left',sensor_queue, 'left lidar'))


        right_lidar_transform = carla.Transform(carla.Location(x=0, y=1.5, z=1.5), carla.Rotation(pitch=0, yaw=90))
        self.right_lidar = self.world.spawn_actor(lidar_bp, right_lidar_transform, attach_to=self.car.ego_vehicle)
        #self.right_lidar.listen(lambda point_cloud: overall_sensor_callback(point_cloud, 'data/lidars/lidar_right/%.6d.ply' % point_cloud.frame, sensor_queue, 'right lidar'))
        self.right_lidar.listen(lambda data: lidar_callback(data,'data/lidars/lidar_right',sensor_queue, 'right lidar'))


        # 获取语义分割摄像机的蓝图
        semseg_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
        semseg_bp.set_attribute('image_size_x', '1920')
        semseg_bp.set_attribute('image_size_y', '1080')
        semseg_bp.set_attribute('fov', '110')
        semseg_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.semseg_front = self.world.spawn_actor(semseg_bp, semseg_transform, attach_to=self.car.ego_vehicle)
        # set the callback function
        self.semseg_front.listen(lambda image: semseg_callback(image, 'data/semseg/front/%.6d.jpg' % image.frame, sensor_queue, 'front semseg'))
        # self.semseg_front.listen(lambda image: image.save_to_disk('data/semseg/front/%.6d.jpg' % image.frame,carla.ColorConverter.CityScapesPalette))
        # sensor_queue.put('front semseg')



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
        transform = carla.Transform(carla.Location(x = 55.54242706298828, y = 306.1947021484375, z = 0.21863333880901337), carla.Rotation(yaw=-0.3095703125))
        self.ego_vehicle = self.world.spawn_actor(self.ego_vehicle_bp, transform)
        self.ego_vehicle.set_autopilot(True, traffic_manager.get_port())
        world.tick()

def output_rotate_matrix(transform_front, s, file_path):
    rotation_front=transform_front.rotation
    location_front=transform_front.location

    R = rotation_matrix_from_euler(rotation_front)
    x, y, z = location_front.x, location_front.y, location_front.z
    R = np.hstack((R, np.array([x, y, z]).reshape(3, 1)))

    R = np.linalg.inv(np.row_stack((R, np.array([0, 0, 0, 1]))))

    transform_s=s.get_transform()
    rotation_s=transform_s.rotation
    location_s=transform_s.location

    R1 = rotation_matrix_from_euler(rotation_s)
    x1, y1, z1 = location_s.x, location_s.y, location_s.z

    R1 = R @ np.row_stack((np.hstack((R1, np.array([x1, y1, z1]).reshape(3, 1))), np.array([0, 0, 0, 1])))
    R1 = R1[:3]
        
    with open(file_path,'w') as f:
        print(R1,file=f)

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
        
        frame_count=10
        f_id = 0

        for _ in range(frame_count):
            print("frame #%d" % f_id)
            f_id += 1
            world.tick()

            camera_pose = sensor.camera_front.get_transform()
            location=camera_pose.location
            x=location.x
            y=location.y
            z=location.z
            rotation=camera_pose.rotation
            R=rotation_matrix_from_euler(rotation)
            pose=np.hstack((R,np.array([[x],[y],[z]])))
            with open('data/poses/rgb_cam2world_5_%d.txt'%f_id,'w') as f:
                print(pose,file=f)

            try:
                for i in range(18):
                    s_name = sensor_queue.get(True, 5.0)
                    print("receiving data from %s" % s_name)

            except Empty:
                print("some sensor data are missing!")
        
        fov =sensor.camera_front.attributes['fov']
        width = int(sensor.camera_front.attributes['image_size_x'])
        height = int(sensor.camera_front.attributes['image_size_y'])
        f = width / (2.0 * np.tan(float(fov) * np.pi / 360.0))
        c_x = width / 2.0
        c_y = height / 2.0

        K = np.array([
            [f, 0, c_x],
            [0, f, c_y],
            [0, 0, 1]
        ])
        with open('data/calib/rgb_intrinsics.txt','w') as f:
            print(K,file=f)

        
        # sensors = world.get_actors().filter('sensor.camera.rgb')

        ## RGB Cameras
        ## 0 - Backright
        ## 1 - Backleft
        ## 2 - Back
        ## 3 - Frontright
        ## 4 - Frontleft
        ## 5 - Front

        print('###', sensor.camera_front.get_transform())

        transform_front = sensor.camera_front.get_transform()
        output_rotate_matrix(transform_front, sensor.camera_back_right, "data/calib/cams/rgb_cam2cam_0.txt")
        output_rotate_matrix(transform_front, sensor.camera_back_left, "data/calib/cams/rgb_cam2cam_1.txt")
        output_rotate_matrix(transform_front, sensor.camera_back, "data/calib/cams/rgb_cam2cam_2.txt")
        output_rotate_matrix(transform_front, sensor.camera_front_right, "data/calib/cams/rgb_cam2cam_3.txt")
        output_rotate_matrix(transform_front, sensor.camera_front_left, "data/calib/cams/rgb_cam2cam_4.txt")
        output_rotate_matrix(transform_front, sensor.camera_front, "data/calib/cams/rgb_cam2cam_5.txt")

        ## LiDARs
        ## 0 - Right LiDAR
        ## 1 - Left LiDAR
        ## 2 - Rear LiDAR
        ## 3 - Front LiDAR
        ## 4 - Top LiDAR

        output_rotate_matrix(transform_front, sensor.right_lidar, "data/calib/lidars/lidar_sensor2cam_0.txt")
        output_rotate_matrix(transform_front, sensor.left_lidar, "data/calib/lidars/lidar_sensor2cam_1.txt")
        output_rotate_matrix(transform_front, sensor.rear_lidar, "data/calib/lidars/lidar_sensor2cam_2.txt")
        output_rotate_matrix(transform_front, sensor.front_lidar, "data/calib/lidars/lidar_sensor2cam_3.txt")
        output_rotate_matrix(transform_front, sensor.lidar_top, "data/calib/lidars/lidar_sensor2cam_4.txt")

    finally:
        sensors = world.get_actors().filter('sensor.*')
        for sensor in sensors:
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

    
