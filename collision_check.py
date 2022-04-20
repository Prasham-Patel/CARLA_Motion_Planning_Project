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
import math
from math import sin, cos, atan2,atan, pi

# Appending paths - DO NOT CHANGE
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    import carla
except IndexError:
    pass

class collision_check():

    def __init__(self, vehicle1, vehicle2):
        self.vehicle1 = vehicle1
        self.vehicle2 = vehicle2

        l, w, h = vehicle1.bounding_box.extent
        theta = atan(w / l)
        self.a1 = l / 2 * cos(theta)
        self.b1 = w / 2 * sin(theta)

        l, w, h = vehicle2.bounding_box.extent
        theta = atan(w / l)
        self.a2 = l / 2 * cos(theta)
        self.b2 = w / 2 * sin(theta)

    def check(self):
        x1, y1 = self.vehicle1.Location.transform.x, self.vehicle1.Location.transform.y
        yaw1 = self.vehicle1.Transform.Rotation.yaw

        x2, y2 = self.vehicle2.Location.transform.x, self.vehicle2.Location.transform.y
        yaw2 = self.vehicle2.Transform.Rotation.yaw

        dist = ((x1-x2)**2 + (y1-y2)**2)**0.5

        theta = atan2(y1-y2,x1-x2)

        r1 = ((self.a1*cos(theta+yaw1))**2 + (self.b1*sin(theta+yaw1))**2)**0.5
        r2 = ((self.a2*cos(theta+yaw2))**2 + (self.b2*sin(theta+yaw2))**2)**0.5

        if r1+r2 >= dist:
            return False
        else:
            return True
