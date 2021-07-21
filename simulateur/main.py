import math, time
from random import random

from robot import *
from graphics import *
from lidar import *
from application import *


t = init_turtle()

basic_obstacles = get_basic_obstacles()

robots = [Robot(1200, 500, 0, True), Robot(-1200, -800, 0, False), Robot(-1200, 800, 0, False), Robot(1200, -500, 0, False)]

time = 0
sex = 0
sey = 0
ser = 0

while 1:

    # robots[0].simulate()
    # robots[1].simulate()
    for r in robots:
        r.simulate()
    obstacles = basic_obstacles + get_dynamic_obstacles(robots)
    for r in robots:
        if r.has_lidar:
            (robot_x, robot_y) = r.get_center()
            robot_r = r.get_rotation()
            (rad_data, cart_data) = get_lidar_data(robot_x, robot_y, robot_r, obstacles)
            r.update_lidar((rad_data, cart_data))

    update_graphics(t, obstacles, robots);

    er = robots[0].r-robots[0].grobot.r
    while er >= 180:er-=360
    while er < -180:er+=360

    sex+=abs(robots[0].x-robots[0].grobot.x)
    sey+=abs(robots[0].y-robots[0].grobot.y)
    ser+=abs(er)
    time+=1

    print("Real :", robots[0].x, robots[0].y, robots[0].r)
    print("Guess:", robots[0].grobot.x, robots[0].grobot.y, robots[0].grobot.r)
    print("Error:", robots[0].x-robots[0].grobot.x, robots[0].y-robots[0].grobot.y, robots[0].r-robots[0].grobot.r)
    print("Average err:", round(sex/time, 2), round(sey/time, 2), round(ser/time, 2))

    if abs(robots[0].x-robots[0].grobot.x) > 100 or abs(robots[0].y-robots[0].grobot.y) > 100:
        input()

    # input()
    # time.sleep(0.1)


# turtle.mainloop()
