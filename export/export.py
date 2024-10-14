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
import argparse
import numpy as np
import random
from tqdm import *
from export_utils import *
from export_sensor_manager import SensorManager
from export_car_manager import CarManager

sensor_queue = Queue()

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    try:
        # print("### START EXPORTING ###")

        args = argparser.parse_args()

        # print("### ARGPARSE OK ###")

        random.seed(2333)
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(20.0)
        world = client.get_world()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.set_hybrid_physics_mode(True)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(233)

        car = CarManager(world, traffic_manager)

        vehicles_list, walkers_list, all_id, all_actors = create_pedestrian_and_vehicle(client, world, traffic_manager, args)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1  # 设置固定帧率
        world.apply_settings(settings)

        weather = carla.WeatherParameters(
            cloudiness=10.000000, 
            precipitation=0.000000, 
            precipitation_deposits=0.000000, 
            wind_intensity=10.000000, 
            sun_azimuth_angle=250.000000, 
            sun_altitude_angle=30.000000, 
            fog_density=10.000000, 
            fog_distance=60.000000, 
            fog_falloff=0.900000, 
            wetness=0.000000, 
            scattering_intensity=1.000000, 
            mie_scattering_scale=0.030000, 
            rayleigh_scattering_scale=0.033100, 
            dust_storm=0.000000
        )

        world.set_weather(weather)

        # print("### SPAWNNING EGO VEHICLE ###")
        sensor = SensorManager(world, car, sensor_queue, [_ % (256 * 256) for _ in vehicles_list], [walker["id"] % (256 * 256) for walker in walkers_list]) # This should be done after generating pedestrians and vehicles because instance segmentation need their ID

        # print("### SPAWNNING EGO VEHICLE DONE ###")
        
        frame_count=150
        f_id = 0

        print(vehicles_list)

        for _ in tqdm(range(frame_count)):#tqdm(range(frame_count)):
            # print("frame #%d" % f_id)
            f_id += 1
            world.tick()

            camera_pose = sensor.cameras[5][0].get_transform()
            location=camera_pose.location
            x=location.x
            y=location.y
            z=location.z
            rotation=camera_pose.rotation
            R=rotation_matrix_from_euler(rotation)
            pose=np.hstack((R,np.array([[x],[y],[z]])))
            try:
                os.makedirs('data/poses')
            except:
                pass
            with open('data/poses/rgb_cam2world_5_%d.txt'%f_id,'w') as f:
                print(pose,file=f)

            try:
                for i in range(29):
                    s_name = sensor_queue.get(True, 5.0)
                    # print("receiving data from %s" % s_name)

            except Empty:
                print("some sensor data are missing!")

            try:
                os.makedirs('data/vehicles')
            except:
                pass
            try:
                os.makedirs('data/walkers')
            except:
                pass

            with open('data/vehicles/vehicle_%d.txt' % f_id, 'w') as f:
                tmp = world.get_actors(vehicles_list)

                # print("### prepare vehicles ###")
                for vehicle in tmp:
                    try:
                        print(vehicle.id, file=f)
                        print(vehicle.type_id, file=f)
                        print(vehicle.get_transform(), file=f)
                    except:
                        pass

            # print("### vehicles output done! ###")

            with open('data/walkers/walker_%d.txt' % f_id, 'w') as f:
                # print("### prepare walkers! ###")
                for walker in walkers_list:
                    # print("### get walker actor! ###")
                    tmp = world.get_actor(walker["id"])
                    # print("### walker Actor get OK ###")
                    print(tmp.id, file=f)
                    # print("### walker ID get OK ###")
                    print(tmp.type_id, file=f)
                    # print("### walker type ID get OK ###")
                    print(tmp.get_transform(), file=f)
                    # print("### walker transform get OK ###")
                    print(tmp.get_bones(), file=f)
                    # print("### walker bones get OK ###")

        
        fov =sensor.cameras[5][0].attributes['fov']
        width = int(sensor.cameras[5][0].attributes['image_size_x'])
        height = int(sensor.cameras[5][0].attributes['image_size_y'])
        f = width / (2.0 * np.tan(float(fov) * np.pi / 360.0))
        c_x = width / 2.0
        c_y = height / 2.0

        K = np.array([
            [f, 0, c_x],
            [0, f, c_y],
            [0, 0, 1]
        ])
        try:
            os.makedirs('data/calib')
        except:
            pass
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

        print('###', sensor.cameras[5][0].get_transform())

        try:
            os.makedirs('data/calib/cams')
        except:
            pass

        transform_front = sensor.cameras[5][0].get_transform()
        output_rotate_matrix(transform_front, sensor.cameras[0][0], "data/calib/cams/rgb_cam2cam_0.txt")
        output_rotate_matrix(transform_front, sensor.cameras[1][0], "data/calib/cams/rgb_cam2cam_1.txt")
        output_rotate_matrix(transform_front, sensor.cameras[2][0], "data/calib/cams/rgb_cam2cam_2.txt")
        output_rotate_matrix(transform_front, sensor.cameras[3][0], "data/calib/cams/rgb_cam2cam_3.txt")
        output_rotate_matrix(transform_front, sensor.cameras[4][0], "data/calib/cams/rgb_cam2cam_4.txt")
        output_rotate_matrix(transform_front, sensor.cameras[5][0], "data/calib/cams/rgb_cam2cam_5.txt")

        ## LiDARs
        ## 0 - Right LiDAR
        ## 1 - Left LiDAR
        ## 2 - Rear LiDAR
        ## 3 - Front LiDAR
        ## 4 - Top LiDAR

        try:
            os.makedirs('data/calib/lidars')
        except:
            pass

        output_rotate_matrix(transform_front, sensor.lidars[0], "data/calib/lidars/lidar_sensor2cam_0.txt")
        output_rotate_matrix(transform_front, sensor.lidars[1], "data/calib/lidars/lidar_sensor2cam_1.txt")
        output_rotate_matrix(transform_front, sensor.lidars[2], "data/calib/lidars/lidar_sensor2cam_2.txt")
        output_rotate_matrix(transform_front, sensor.lidars[3], "data/calib/lidars/lidar_sensor2cam_3.txt")
        output_rotate_matrix(transform_front, sensor.lidars[4], "data/calib/lidars/lidar_sensor2cam_4.txt")

    finally:
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
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

    
