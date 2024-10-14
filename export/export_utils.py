import glob
import sys
import os
import random
import logging
import numpy as np
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def get_actor_blueprints(world, filter, generation):
    '''
    Get the blueprints of the actors that match the filter and generation
    '''
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []
    
def get_camera_matrix(transform):
    rotation_matrix = transform.get_matrix()[:3, :3]
    translation_vector = transform.location
    camera_matrix = np.hstack((rotation_matrix, np.array(translation_vector).reshape(3, 1)))
    return camera_matrix

def rotation_matrix_from_euler(rotation: carla.Rotation):
    '''
    Convert a CARLA rotation to a 3x3 rotation matrix.
    '''
    # Convert degrees to radians
    return np.array(carla.Transform(carla.Location(0, 0, 0), rotation).get_matrix())[:3, :3]

def create_camera_set(world, image_size_x, image_size_y, fov, location, rotation, attach_to):
    '''
    Create a group of: RGB camera, Depth camera, Instance and Object segmentation camera on the given position
    Returning them in the order of: RGB, Depth, ISC, OSC
    '''
    # Get needed blueprints
    blueprint_library = world.get_blueprint_library()
    rgb_bp = blueprint_library.find('sensor.camera.rgb')
    depth_bp = blueprint_library.find('sensor.camera.depth')
    is_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
    os_bp = blueprint_library.find('sensor.camera.instance_segmentation')

    # Set the image size and field of view
    rgb_bp.set_attribute('image_size_x', str(image_size_x))
    rgb_bp.set_attribute('image_size_y', str(image_size_y))
    rgb_bp.set_attribute('fov', str(fov))
    depth_bp.set_attribute('image_size_x', str(image_size_x))
    depth_bp.set_attribute('image_size_y', str(image_size_y))
    depth_bp.set_attribute('fov', str(fov))
    is_bp.set_attribute('image_size_x', str(image_size_x))
    is_bp.set_attribute('image_size_y', str(image_size_y))
    is_bp.set_attribute('fov', str(fov))
    os_bp.set_attribute('image_size_x', str(image_size_x))
    os_bp.set_attribute('image_size_y', str(image_size_y))
    os_bp.set_attribute('fov', str(fov))

    # Create the cameras
    rgb_camera = world.spawn_actor(rgb_bp, carla.Transform(location, rotation), attach_to=attach_to)
    depth_camera = world.spawn_actor(depth_bp, carla.Transform(location, rotation), attach_to=attach_to)
    is_camera = world.spawn_actor(is_bp, carla.Transform(location, rotation), attach_to=attach_to)
    os_camera = world.spawn_actor(os_bp, carla.Transform(location, rotation), attach_to=attach_to)

    return rgb_camera, depth_camera, is_camera, os_camera

def create_ego_camera_sensors(world, image_size_x, image_size_y, fov, locations, rotations, attach_to, callbacks):
    '''
    Create a group of: RGB camera, Depth camera, Instance and Object segmentation camera on the given position
    '''
    set_num = len(locations)
    assert set_num == len(rotations) == len(callbacks), "The number of locations, rotations and callbacks must be the same."
    sensors = []
    for i in range(set_num):
        rgb_camera, depth_camera, is_camera, os_camera = create_camera_set(world, image_size_x, image_size_y, fov, locations[i], rotations[i], attach_to)
        rgb_camera.listen(callbacks[i][0])
        depth_camera.listen(callbacks[i][1])
        is_camera.listen(callbacks[i][2])
        os_camera.listen(callbacks[i][3])
        sensors.append([rgb_camera, depth_camera, is_camera, os_camera])
    return sensors[::-1] # reversed order

def create_ego_lidar_set(world, channels, lrange, lnoise, rotation_frequency, points_per_second, locations, rotations, attach_to, callbacks):
    '''
    Create a group of LIDAR sensors on the given position of the ego vehicle
    '''
    # Get needed blueprints
    blueprint_library = world.get_blueprint_library()
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

    # Set the channels, range and noise
    lidar_bp.set_attribute('channels', str(channels))
    lidar_bp.set_attribute('range', str(lrange))
    lidar_bp.set_attribute('noise_stddev', str(lnoise))
    lidar_bp.set_attribute('rotation_frequency', str(rotation_frequency))
    lidar_bp.set_attribute('points_per_second', str(points_per_second))

    # Create the lidars
    lidars = []
    for i in range(len(locations)):
        lidar = world.spawn_actor(lidar_bp, carla.Transform(locations[i], rotations[i]), attach_to=attach_to)
        lidar.listen(callbacks[i])
        lidars.append(lidar)

    return lidars[::-1] # reversed order

def create_pedestrian_and_vehicle(client, world, traffic_manager, args):
    '''
    Create a group of pedestrians and vehicles in the world
    '''
    vehicles_list = []
    walkers_list = []
    all_id = []
    random.seed(2333)

    # print("### GET BLUEPRINTS ###")

    blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
    blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)

    # print("### GET BLUEPRINTS DONE ###")

    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    # print("### READY TO CHOOSE SPAWNPOINTS ###")

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if args.number_of_vehicles < number_of_spawn_points:
        random.shuffle(spawn_points)
    elif args.number_of_vehicles > number_of_spawn_points:
        msg = 'requested %d vehicles, but could only find %d spawn points'
        args.number_of_vehicles = number_of_spawn_points

    # @todo cannot import these directly.
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    # print("### START SPAWNING VEHICLES ###")

    # --------------
    # Spawn vehicles
    # --------------
    batch = []
    hero = args.hero
    for n, transform in enumerate(spawn_points):
        print("### SPAWNNING %d ###" % n)
        if n >= args.number_of_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if hero:
            blueprint.set_attribute('role_name', 'hero')
            hero = False
        else:
            blueprint.set_attribute('role_name', 'autopilot')

        # spawn the cars and set their autopilot and light state all together
        batch.append(SpawnActor(blueprint, transform)
            .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)

    # print("### SPAWNNING VEHICLES OK ###")

    # -------------
    # Spawn Walkers
    # -------------
    # some settings
    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
    world.set_pedestrians_seed(2333)
    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(args.number_of_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            # print("### ERROR ###")
            logging.error(results[i].error)
        else:
            # print("### APPENDING ###")
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    # 3. we spawn the walker controller
    # print("### SPAWN WALKER OK ###")
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    world.tick()
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            # print("### ERROR ###")
            logging.error(results[i].error)
        else:
            # print("### EDITTING ###")
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put together the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)
    # print("### SPAWN CONTROLLER OK ###")

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    world.tick()
    # print("### TICKING ###")

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    # print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))
    return vehicles_list, walkers_list, all_id, all_actors

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