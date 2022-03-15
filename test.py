#!/usr/bin/env python

import glob
import os
import sys
import matplotlib.pyplot as plt
import networkx as nx

try:
    # Appending paths - DO NOT CHANGE
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import cv2
import random
import time
import numpy as np
import agents.navigation.global_route_planner_dao as DAO
import agents.navigation.global_route_planner as planner

def main_test():
    try:
        client = carla.Client('localhost', 2000)     # port setting
        client.set_timeout(10.0)     # increase time-out buffer if system is slow in responding

        world = client.get_world()      # CARLA World entity
        world = client.load_world('Town01')
        map = world.get_map()
        # print(map.get_topology()[0][0])
        # print(map.get_topology()[0][1])

        raw_graph_data = DAO.GlobalRoutePlannerDAO(map, 1)
        nx_builder = planner.GlobalRoutePlanner(raw_graph_data)
        nx_builder.setup()
        graph, id_map, road_id_to_edge = nx_builder._build_graph()
        # nx.draw(graph)
        plt.figure(figsize=(10, 9))
        nx.draw_spectral(graph, with_labels=True, node_size = 0.5)
        plt.savefig("Town_map.png")
        print(road_id_to_edge)


        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # getting actor
        print("1")
        bp = random.choice(blueprint_library.filter('vehicle'))

        # Now we need to give an initial transform to the vehicle. We choose a
        # random transform from the list of recommended spawn points of the map.
        transform = random.choice(world.get_map().get_spawn_points())
        print(transform)
        time.sleep(3)

        vehicle = world.spawn_actor(bp, transform)
        control = carla.VehicleControl()
        i = 0
        while i < 10000:
           control.throttle = 0.5
           # control
           vehicle.apply_control(control)
           print(vehicle.get_location())
           # print(vehicle.get_acceleration())
           i += 1

        # camera = carla.sensor.Camera('MyCamera', PostProcessing='SceneFinal')
        # camera.set(FOV=90.0)
        # camera.set_image_size(800, 600)
        # camera.set_position(x=0.30, y=0, z=1.30)
        # camera.set_rotation(pitch=0, yaw=0, roll=0)





    finally:

        print('destroying actors')
        # camera.destroy()
        # client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')

#
if __name__ == '__main__':
    main_test()
# main_test()
