#!/usr/bin/env python

import glob
import os
import sys
import matplotlib.pyplot as plt
import networkx as nx

import cv2
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
        self.carla_world = self.client.get_world()

        print("WORLD READY")

        # Set up the simulator in synchronous mode
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True  # Enables synchronous mode
        settings.fixed_delta_seconds = 0.05
        self.carla_world.apply_settings(settings)

        # We will also set up the spectator so we can see what we do
        self.spectator = self.carla_world.get_spectator()

        self.vehicles = []

    def spawn_vehicle(self, vehicle_name, spawn_point):
        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = self.carla_world.get_blueprint_library()
        bp = blueprint_library.filter(vehicle_name)[0]
        vehicle = self.carla_world.spawn_actor(bp, spawn_point)
        self.vehicles.append(vehicle)

    def spawn_ego_vehicle(self, vehicle_name, spawn_point):
        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = self.carla_world.get_blueprint_library()
        bp = blueprint_library.filter(vehicle_name)[0]
        self.ego_vehicle = self.carla_world.spawn_actor(bp, spawn_point)
        # print(self.ego_vehicle.get_location())



    # def display(self):
    #     # Update the display
    #     gameDisplay.blit(renderObject.surface, (0, 0))
    #     pygame.display.flip()

if __name__ == '__main__':

    CARLA_world = world('Town03')
    spawn_point1 = carla.Transform(carla.Location(x=396, y=105, z=0.3), carla.Rotation(yaw=-90))
    CARLA_world.spawn_ego_vehicle('model3', spawn_point1)
    spawn_point2 = carla.Transform(carla.Location(x=396, y=60, z=0.3), carla.Rotation(yaw=-90))
    CARLA_world.spawn_vehicle('model3', spawn_point2)
    spawn_point3 = carla.Transform(carla.Location(x=396, y=40, z=0.3), carla.Rotation(yaw=-90))
    CARLA_world.spawn_vehicle('model3', spawn_point3)

    # WINDOW DISPLAY CODE
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

    while True:
        CARLA_world.carla_world.tick()  # DO NOT CHANGE | CANNOT RUN SIMULATION WITHOUT TICK()
        gameDisplay.fill((0, 0, 0))
        gameDisplay.blit(renderObject.surface, (0, 0))
        pygame.display.flip()

    # Stop camera and quit PyGame after exiting game loop
    camera.stop()
    pygame.quit()
