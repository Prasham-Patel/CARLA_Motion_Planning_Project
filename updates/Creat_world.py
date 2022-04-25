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

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
client.set_timeout(30.0)  # increase time-out buffer if system is slow in responding
world = client.get_world()
world = client.load_world('Town01')

# KEEP COMMENTED FOR NOW

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# # Set up the TM in synchronous mode
# traffic_manager = client.get_trafficmanager()
# traffic_manager.set_synchronous_mode(True)
#
# # Set a seed so behaviour can be repeated if necessary
# traffic_manager.set_random_device_seed(0)
# random.seed(0)


# We will aslo set up the spectator so we can see what we do
# spectator = world.get_spectator()

# Retrieve the map's spawn points
spawn_points = world.get_map().get_spawn_points()

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

# The world contains the list blueprints that we can use for adding new
# actors into the simulation.
blueprint_library = world.get_blueprint_library()
# getting actor
bp = blueprint_library.filter('model3')[0]
transform = random.choice(world.get_map().get_spawn_points())
print(transform)
time.sleep(3)

vehicle = world.spawn_actor(bp, transform)
print(vehicle.get_location())

# Initialise the camera floating behind the vehicle
camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

# Get camera dimensions
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

# Instantiate objects for rendering and vehicle control
renderObject = RenderObject(image_w, image_h)

# Start camera with PyGame callback
camera.listen(lambda image: pygame_callback(image, renderObject))

# Initialise the display
pygame.init()
gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
# Draw black to the display
gameDisplay.fill((0,0,0))
gameDisplay.blit(renderObject.surface, (0,0))
pygame.display.flip()

# Game loop
crashed = False
i = 0

while True:
    # control = carla.VehicleControl()
    # i += 1
    #
    # control.throttle = 0.5
    # # control
    # vehicle.apply_control(control)
    # print(vehicle.get_location())
    # Advance the simulation time
    world.tick()
    # Update the display
    gameDisplay.fill((0, 0, 0))
    gameDisplay.blit(renderObject.surface, (0,0))
    pygame.display.flip()
    # Process the current control state
    # controlObject.process_control()
    # Collect key press events
#     for event in pygame.event.get():
# # Update PyGame window
#         gameDisplay.fill((0,0,0))
#         gameDisplay.blit(renderObject.surface, (0,0))
#         pygame.display.flip()
    #     # If the window is closed, break the while loop
    #     if event.type == pygame.QUIT:
    #         crashed = True
    #
    #     # Parse effect of key press event on control state
    #     # controlObject.parse_control(event)
    #     if True:
    #         # TAB key switches vehicle
    #         if True:
    #             # ego_vehicle.set_autopilot(True)
    #             # ego_vehicle = random.choice(vehicles)
    #             # Ensure vehicle is still alive (might have been destroyed)
    #             if vehicle.is_alive:
    #                 # Stop and remove the camera
    #                 camera.stop()
    #                 camera.destroy()
    #
    #                 # Spawn new camera and attach to new vehicle
    #                 # controlObject = ControlObject(ego_vehicle)
    #                 camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    #                 camera.listen(lambda image: pygame_callback(image, renderObject))
    #
    #                 # Update PyGame window
    #                 gameDisplay.fill((0,0,0))
    #                 gameDisplay.blit(renderObject.surface, (0,0))
    #                 pygame.display.flip()

# Stop camera and quit PyGame after exiting game loop
camera.stop()
pygame.quit()
