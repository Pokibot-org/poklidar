import math, time
from random import random

from robot import *
from graphics import *
from lidar import *
from application import *


t = init_turtle();

basic_obstacles = get_basic_obstacles()

robots = [Robot(1200, 0, 0, True)]#, Robot(-1200, 800, 0, False)]

while 1:

    robots[0].simulate()
    # for r in robots:
        # r.simulate()
    obstacles = basic_obstacles + get_dynamic_obstacles(robots)
    for r in robots:
        if r.has_lidar:
            (robot_x, robot_y) = r.get_center()
            robot_r = r.get_rotation()
            (rad_data, cart_data) = get_lidar_data(robot_x, robot_y, robot_r, obstacles)
            r.update_lidar((rad_data, cart_data))

    update_graphics(t, obstacles, robots);
    # time.sleep(0.1)


# turtle.mainloop()
