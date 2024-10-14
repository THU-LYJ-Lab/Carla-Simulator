import numpy as np
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

def overall_sensor_callback(sensor_data, disk_path, sensor_queue, sensor_name):
    # if sensor_data.frame % 10 == 11:
    sensor_data.save_to_disk(disk_path)
    sensor_queue.put(sensor_name)

def semseg_callback(sensor_data, disk_path, sensor_queue, sensor_name):
    array = np.frombuffer(sensor_data.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (sensor_data.height, sensor_data.width, 4)) 
    semantic_image = array[:, :, 2]
    if not os.path.exists(disk_path):
        os.makedirs(disk_path)

    filename = os.path.join(disk_path, f'semseg_frame_{sensor_data.frame}.npz')
    np.savez_compressed(filename, arr_0=semantic_image)
    sensor_queue.put(sensor_name)

def insseg_callback(sensor_data, disk_path, sensor_queue, sensor_name, vehicle_ids, walker_ids):
    array = np.frombuffer(sensor_data.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (sensor_data.height, sensor_data.width, 4))
    instance_image = array[:, :, :3][:, :, ::-1]
    if not os.path.exists(disk_path):
        os.makedirs(disk_path)
    
    filename = os.path.join(disk_path, f'insseg_frame_{sensor_data.frame}.npz')
    np.savez_compressed(filename, arr_0=instance_image)
    sensor_data.save_to_disk(disk_path + "/" + str(sensor_data.frame) + ".png")

    actorid_image = (instance_image.astype(np.int32)[:,:,1] + instance_image.astype(np.int32)[:,:,2]*256).astype(np.int32)
    empty_image = np.zeros(actorid_image.shape).astype(np.int8)
    for i in range(len(vehicle_ids)):
        empty_image[actorid_image == vehicle_ids[i]] = 1
    for i in range(len(walker_ids)):
        empty_image[actorid_image == walker_ids[i]] = 1
        
    filename = os.path.join(disk_path, f'dynamic_object_mask_frame_{sensor_data.frame}.npz')
    np.savez_compressed(filename, arr_0 = empty_image)
    sensor_queue.put(sensor_name)

def lidar_callback(data,save_folder,sensor_queue, sensor_name):
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
    sensor_queue.put(sensor_name)