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

class CarManager(object):
    def __init__(self, world, traffic_manager):
        self.world = world
        self.ego_vehicle = None
        blueprint_library = world.get_blueprint_library()
        self.ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        self.ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # transform = random.choice(world.get_map().get_spawn_points())

        route_points = [
            carla.Location(x=6050.015625 / 100,y=13420.777344 / 100,z=0.032131 / 100),
            carla.Location(x=-4367.756836 / 100,y=13364.735352 / 100,z=0.033600 / 100)

            #carla.Location(x=-5834.045410/100,y=-19472.632812/100,z=0.331562/100) 
            # carla.Location(x=-6193.812012 / 100.0,y=-329.354614 / 100.0,z=0.324142 / 100.0),
            # carla.Location(x=2197.394775 / 100,y=644.150940 / 100,z=-0.098522 / 100),
            # carla.Location(x=8820.413086 / 100,y=31295.304688 / 100,z=0.033731 / 100),
        ]
        
        transform = carla.Transform(carla.Location(x=6050.015625 / 100,y=13420.777344 / 100,z=0.032131 / 100), carla.Rotation (pitch=-0.016208,yaw=-178.680588,roll=-0.000007))
        self.ego_vehicle = self.world.spawn_actor(self.ego_vehicle_bp, transform)
        self.ego_vehicle.set_autopilot(True, traffic_manager.get_port())
        traffic_manager.ignore_lights_percentage(self.ego_vehicle,100)
        traffic_manager.set_path(self.ego_vehicle, route_points)
        world.tick()