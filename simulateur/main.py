import math, time
from random import random

from robot import *
from graphics import *
from lidar import *
from application import *
from HLS_LFCD2 import *



# lidar = HLS_LFCD2()
# lidar.open()
# lidar.start()

t = init_turtle()

basic_obstacles = get_basic_obstacles()

robots = [
    Robot(0, 0, 0, True),
    Robot(-1200, -800, 0, False),
    Robot(-1200, 800, 0, False),
    Robot(1200, -500, 0, False)
]

jean_michels = []
if JEAN_MICHEL_CASSE_COUILLES:
    jean_michels = [
        Jean_Michel(-2000, +1000+400, -1000, +1000+400),
        Jean_Michel(800, +1000+200, 2200, +1000+600),
        Jean_Michel(+1500+200, -600, +1500+600, +600),
        Jean_Michel(-1500-600, -800, -1500-200, +800)
    ]

time = 0
sex = 0
sey = 0
ser = 0

while 1:



    print("======================================================================")
    print("======================================================================")
    print("======================================================================")

    for j in jean_michels:
        j.simulate()

    # robots[0].simulate()
    # robots[1].simulate()
    # for r in robots:
    #     r.simulate()
    #     r.tick(1)
    # print("Real :", robots[0].x, robots[0].y, robots[0].r)
    # obstacles = basic_obstacles + get_dynamic_obstacles(robots, jean_michels)
    # for r in robots:
    #     if r.has_lidar:
    #         (robot_x, robot_y) = r.get_center()
    #         robot_r = r.get_rotation()
    #         rad_data = get_lidar_data(robot_x, robot_y, robot_r, obstacles)
    #         # rad_data = lidar.get_one_scan()
    #         r.update_lidar(rad_data, True)

    for r in robots:
        r.simulate()
    steps = LIDAR_POINTS_PER_TURN//LIDAR_ANGLE_PER_STEP
    rad_data = []
    # rad_data = lidar.get_one_scan()
    for i in range(steps):
        for r in robots:
            r.tick((i+1)/steps)
        obstacles = basic_obstacles + get_dynamic_obstacles(robots, jean_michels)
        rad_data += get_lidar_data(robots[0].x, robots[0].y, robots[0].r, obstacles, i*LIDAR_ANGLE_PER_STEP, (i+1)*LIDAR_ANGLE_PER_STEP)

    print("XXXXXXX", steps, len(rad_data))
    robots[0].update_lidar(rad_data, True, 0)




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
    print("Error:", robots[0].grobot.x-robots[0].x, robots[0].grobot.y-robots[0].y, robots[0].grobot.r-robots[0].r)
    print("Average err:", round(sex/time, 2), round(sey/time, 2), round(ser/time, 2))

    if abs(robots[0].x-robots[0].grobot.x) > 100 or abs(robots[0].y-robots[0].grobot.y) > 100:
        input()

    # input()
    # time.sleep(0.1)


# turtle.mainloop()
