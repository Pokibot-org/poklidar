import math
from application import *

from graphics import mm2px

# Point to track
class PTT:
    def __init__(self):
        self.x = 0
        self.y = 0


class Robot:
    def __init__(self, x, y, r, has_lidar):
        self.x = x # Position X
        self.y = y # Position Y
        self.r = r # Rotation R

        self.guess_x = x
        self.guess_y = y
        self.guess_r = r

        self.ptt_list = []

        self.dx = 1000/10
        self.dy = 1000/10

        self.has_lidar = has_lidar
        self.lidar_data = None

        self.radius = 1200/(2*math.pi)

        self.mode_xy = 1

        self.grobot = GRobot(x, y, r)

        self.time = 0
    def get_center(self):
        return (self.x, self.y)
    def get_rotation(self):
        return self.r
    def get_radius(self):
        return self.radius

    def simulate(self):

        # self.grobot.x = self.x
        # self.grobot.y = self.y
        # self.grobot.r = self.r

        if self.time != 0:

            self.x+=self.dx
            self.y+=self.dy

            if self.mode_xy == 0:
                if self.x < -1500+self.radius and self.dx < 0:
                    self.dy = -self.dx
                    self.dx = 0
                elif self.x > +1500-self.radius and self.dx > 0:
                    self.dy = -self.dx
                    self.dx = 0
                elif self.y < -1000+self.radius and self.dy < 0:
                    self.dx = self.dy
                    self.dy = 0
                elif self.y > +1000-self.radius and self.dy > 0:
                    self.dx = self.dy
                    self.dy = 0
            elif self.mode_xy == 1:
                if self.x < -1500+self.radius:
                    self.dx = -self.dx
                elif self.x > +1500-self.radius:
                    self.dx = -self.dx
                elif self.y < -1000+self.radius:
                    self.dy = -self.dy
                elif self.y > +1000-self.radius:
                    self.dy = -self.dy


            self.r = (self.r+360/10)%360

        self.time+=1

    def get_lidar_data(self):
        return self.lidar_data
    def update_lidar(self, lidar_data):
        (rad_data, _) = lidar_data
        prev_rad_data = rad_data
        if self.lidar_data != None:
            (prev_rad_data, _) = self.lidar_data
        self.lidar_results = solve_lidar(rad_data, prev_rad_data, self.grobot)
        self.lidar_data = lidar_data

    def has_lidar(self):
        return self.has_lidar
