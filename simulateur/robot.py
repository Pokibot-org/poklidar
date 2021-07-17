import math
from application import *

from graphics import mm2px

class Robot:
    def __init__(self, x, y, r, has_lidar):
        self.x = x # Position X
        self.y = y # Position Y
        self.r = r # Rotation R

        self.dx = 10
        self.dy = 0

        self.has_lidar = has_lidar
        self.lidar_data = None

        self.radius = 1200/(2*math.pi)
    def get_center(self):
        return (self.x, self.y)
    def get_rotation(self):
        return self.r
    def get_radius(self):
        return self.radius

    def simulate(self):

        self.x+=self.dx
        self.y+=self.dy
        if self.x < -1500+self.radius or self.x > 1500-self.radius:
            self.dx = -self.dx

        # self.r = (self.r+1)%360

    def get_lidar_data(self):
        return self.lidar_data
    def update_lidar(self, lidar_data):
        self.lidar_data = lidar_data
        (rad_data, cart_data) = lidar_data
        self.lidar_results = solve_lidar(rad_data)

    def has_lidar(self):
        return self.has_lidar
