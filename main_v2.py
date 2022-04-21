#!/usr/bin/env python

import glob
import os
import sys
import matplotlib.pyplot as plt
import networkx as nx

# import cv2
import random
import time
import numpy as np
import pygame


# Appending paths - DO NOT CHANGE
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    import carla
except IndexError:
    pass

# DO NOT TOUCH !!!
# Render object to keep and pass the PyGame surface
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))

# DO NOT TOUCH !!!
# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    # cv2.imshow("img", img)
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))

class world():

    def __init__(self, Town):
        # Connect to the client and retrieve the world object
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(120.0)  # increase time-out buffer if system is slow in responding
        self.carla_world = self.client.load_world(Town)
        self.map = self.carla_world.get_map()

        print("WORLD READY")

        # Set up the simulator in synchronous mode
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True  # Enables synchronous mode
        settings.fixed_delta_seconds = 0.05
        self.carla_world.apply_settings(settings)

        # We will also set up the spectator so we can see what we do
        self.spectator = self.carla_world.get_spectator()

        self.vehicles = []
        self.ego_vehicle = None


if __name__ == '__main__':
    # Connect to the client and retrieve the world object
    # client = carla.Client('localhost', 2000)
    # client.set_timeout(120.0)  # increase time-out buffer if system is slow in responding
    # # world = client.get_world()
    # world = client.load_world('Town01')
    # map = world.get_map()
    # print("World Ready")

    # KEEP COMMENTED FOR NOW

    # Set up the simulator in synchronous mode
    # settings = world.get_settings()
    # settings.synchronous_mode = True  # Enables synchronous mode
    # settings.fixed_delta_seconds = 0.05
    # world.apply_settings(settings)

    CARLA_world = world('Town01')
    # Retrieve the map's spawn points
    spawn_points = CARLA_world.map.get_spawn_points()

    # VEHICLE 1 SPAWN
    spawn_point1 = carla.Transform(carla.Location(x=396, y=40, z=0.3), carla.Rotation(yaw=-90))
    blueprint_library = CARLA_world.carla_world.get_blueprint_library()
    bp1 = blueprint_library.filter("model3")[0]
    vehicle1 = CARLA_world.carla_world.spawn_actor(bp1, spawn_point1)
    CARLA_world.vehicles.append(vehicle1)

    # VEHICLE 2 SPAWN
    spawn_point2 = carla.Transform(carla.Location(x=396, y=60, z=0.3), carla.Rotation(yaw=-90))
    bp2 = blueprint_library.filter("model3")[0]
    vehicle2 = CARLA_world.carla_world.spawn_actor(bp2, spawn_point2)
    CARLA_world.vehicles.append(vehicle2)

    # EGO VEHICLE SPAWN
    spawn_point3 = carla.Transform(carla.Location(x=396, y=105, z=0.3), carla.Rotation(yaw=-90))
    bp3 = blueprint_library.filter("cybertruck")[0]
    CARLA_world.ego_vehicle = CARLA_world.carla_world.spawn_actor(bp3, spawn_point3)

    # Initialise the camera floating behind the vehicle
    camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
    camera_bp = CARLA_world.carla_world.get_blueprint_library().find('sensor.camera.rgb')
    camera = CARLA_world.carla_world.spawn_actor(camera_bp, camera_init_trans, attach_to=CARLA_world.ego_vehicle)

    # Get camera dimensions
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()

    # Instantiate objects for rendering and vehicle control
    renderObject = RenderObject(image_w, image_h)

    # Start camera with PyGame callback
    camera.listen(lambda image: pygame_callback(image, renderObject))

    # Initialise the display
    pygame.init()
    gameDisplay = pygame.display.set_mode((image_w, image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
    # Draw black to the display
    gameDisplay.fill((0, 0, 0))
    gameDisplay.blit(renderObject.surface, (0, 0))
    pygame.display.flip()

    # map = world.get_map()
    waypoint01 = CARLA_world.map.get_waypoint(carla.Location(x=396, y=105), project_to_road=False)
    print(    waypoint01.transform.location.x,     waypoint01.transform.location.y)
    # Game loop
    while True:
        control = carla.VehicleControl()
        # i += 1

        control.throttle = 0.5
        # control
        CARLA_world.ego_vehicle.apply_control(control)
        trans = CARLA_world.ego_vehicle.get_transform()
        print(trans.rotation.yaw)
        # Advance the simulation time
        CARLA_world.carla_world.tick()
        # Update the display
        gameDisplay.fill((0, 0, 0))
        gameDisplay.blit(renderObject.surface, (0, 0))
        pygame.display.flip()

    camera.stop()
    pygame.quit()
