import turtle
from config import *
import math

# Milimeter to pixel conversion
def mm2px(mm):
    return PIXEL_PER_METER*mm/1000

def draw_table(tl):
    offset = tl.pos()
    tl.goto(offset + (mm2px(-TABLE_WIDTH//2), mm2px(-TABLE_HEIGHT//2)))
    tl.pendown()
    tl.goto(offset + (mm2px(+TABLE_WIDTH//2), mm2px(-TABLE_HEIGHT//2)))
    tl.goto(offset + (mm2px(+TABLE_WIDTH//2), mm2px(+TABLE_HEIGHT//2)))
    tl.goto(offset + (mm2px(-TABLE_WIDTH//2), mm2px(+TABLE_HEIGHT//2)))
    tl.goto(offset + (mm2px(-TABLE_WIDTH//2), mm2px(-TABLE_HEIGHT//2)))
    tl.penup()
    tl.goto(offset)

def draw_obstacles(tl, obstacles):
    offset = tl.pos()
    tl.color('red', 'red')
    for o in obstacles:
        (x, y) = o.coords[0]
        tl.goto(offset + (mm2px(x), mm2px(y)))
        tl.pendown()
        tl.begin_fill()
        for (x, y) in o.coords:
            tl.goto(offset + (mm2px(x), mm2px(y)))
        tl.end_fill()
        tl.penup()
    tl.color('black', 'black')
    tl.goto(offset)

# def draw_lidar_view(tl, xc, yc, data):
#     offset = tl.pos()
#     tl.color('green', 'green')
#     for (x, y) in data:
#         if (x, y) != (xc, yc):
#             tl.goto(offset + (mm2px(xc), mm2px(yc)))
#             tl.pendown()
#             tl.goto(offset + (mm2px(x), mm2px(y)))
#             tl.penup()
#     tl.color('black', 'black')
#     tl.goto(offset)

def draw_lidar_view2(tl, xc, yc, data, r0=0):
    offset = tl.pos()
    tl.color('blue', 'blue')

    for i in range(LIDAR_POINTS_PER_TURN):
        angle = r0*math.pi/180
        if LIDAR_ROTATE_CLOCKWISE:
            angle += ((-i)*2*math.pi/LIDAR_POINTS_PER_TURN)
        else:
            angle += ((+i)*2*math.pi/LIDAR_POINTS_PER_TURN)
        x = data[i]*math.cos(angle)+xc
        y = data[i]*math.sin(angle)+yc
        tl.goto(offset + (mm2px(xc), mm2px(yc)))
        tl.pendown()
        tl.goto(offset + (mm2px(x), mm2px(y)))
        tl.penup()

    tl.color('black', 'black')
    tl.goto(offset)

def draw_robot(tl, robot):
    offset = tl.pos()
    (x, y) = robot.get_center()
    r = robot.get_rotation()
    radius = robot.get_radius()
    tl.goto(offset + (mm2px(x), mm2px(y-radius)))
    tl.pendown()
    tl.circle(mm2px(radius))
    tl.penup()

    (tx, ty) = (x+radius*math.cos(r*math.pi/180), y+radius*math.sin(r*math.pi/180))
    tl.goto(offset + (mm2px(tx), mm2px(ty)-5))
    tl.pendown()
    tl.circle(5)
    tl.penup()

    tl.goto(offset)
    if robot.has_lidar:
        rad_data = robot.lidar_data
        # draw_lidar_view(tl, x, y, cart_data)
        draw_lidar_view2(tl, x, y, rad_data, robot.r)


def draw_raw_lidar(tl, rad_data, lidar_results, robot):
    offset = tl.pos()

    # # Draw distance vue
    # factor = 0.5
    # tl.color('blue', 'blue')
    # tl.goto(offset + (0, mm2px(factor*rad_data[0])))
    # tl.pendown()
    # for i in range(LIDAR_POINTS_PER_TURN):
    #     angle = math.pi/2
    #     if LIDAR_ROTATE_CLOCKWISE:
    #         angle += -2*math.pi*i/LIDAR_POINTS_PER_TURN
    #     else:
    #         angle += 2*math.pi*i/LIDAR_POINTS_PER_TURN
    #     j = (i+1)%LIDAR_POINTS_PER_TURN
    #     (x, y) = (factor*rad_data[j]*math.cos(angle), factor*rad_data[j]*math.sin(angle))
    #     tl.goto(offset + (mm2px(x), mm2px(y)))
    # tl.penup()

    # Draw delta vue
    factor = 0.5
    (delta, poi) = lidar_results
    tl.color('orange', 'orange')
    tl.goto(offset + (0, mm2px(factor*abs(delta[0]))))
    tl.pendown()
    for i in range(LIDAR_POINTS_PER_TURN):
        angle = math.pi/2
        if LIDAR_ROTATE_CLOCKWISE:
            angle += -2*math.pi*i/LIDAR_POINTS_PER_TURN
        else:
            angle += 2*math.pi*i/LIDAR_POINTS_PER_TURN
        j = (i+1)%LIDAR_POINTS_PER_TURN
        (x, y) = (factor*abs(delta[j])*math.cos(angle), factor*abs(delta[j])*math.sin(angle))
        tl.goto(offset + (mm2px(x), mm2px(y)))
    tl.penup()

    tl.color('orange', 'orange')
    tl.pensize(3)
    for i in range(-LIDAR_POINTS_PER_TURN//2+1, LIDAR_POINTS_PER_TURN//2, 1):
        (x, y) = (i*WINDOW_WIDTH/LIDAR_POINTS_PER_TURN, -WINDOW_HEIGHT//2+100)
        tl.goto(x, y)
        tl.pendown()
        tl.goto(x, y+0.02*delta[i])
        tl.penup()

    # Draw POI
    factor = 0.5
    tl.pensize(1)
    for p_i in range(len(poi)):
        p = poi[p_i]
        (i0, i1, average, size) = p.get_all()
        angle = math.pi/2
        i = (i1-1+i0)/2
        # if i0 > i1:
        #     i = (i1+LIDAR_POINTS_PER_TURN-1+i0)/2
        if LIDAR_ROTATE_CLOCKWISE:
            angle += -2*math.pi*i/LIDAR_POINTS_PER_TURN
        else:
            angle += 2*math.pi*i/LIDAR_POINTS_PER_TURN
        (x, y) = (factor*average*math.cos(angle), factor*average*math.sin(angle))
        tl.goto(offset + (mm2px(x), mm2px(y)-5))
        if p.has_interesting_size():
            tl.color('purple', 'purple')
        else:
            tl.color('orange', 'orange')
        tl.pendown()
        if p.has_interesting_size():
            tl.begin_fill()
        tl.circle(5)
        if p.has_interesting_size():
            tl.end_fill()
        tl.penup()

    (cx, cy) = TABLE_CENTER
    factor = 1
    for p_i in range(len(poi)):
        p = poi[p_i]
        (i0, i1, average, size) = p.get_all()
        angle = robot.r*math.pi/180
        i = (i1-1+i0)/2
        if LIDAR_ROTATE_CLOCKWISE:
            angle += (-2*math.pi*i)/LIDAR_POINTS_PER_TURN
        else:
            angle += (2*math.pi*i)/LIDAR_POINTS_PER_TURN
        (x, y) = (factor*average*math.cos(angle), factor*average*math.sin(angle))
        tl.goto((mm2px(x+robot.x)+cx, mm2px(y+robot.y)-5+cy))
        if p.has_interesting_size():
            tl.color('purple', 'purple')
        else:
            tl.color('orange', 'orange')
        tl.pendown()
        #if p.has_interesting_size():
        if p.has_interesting_size():
            tl.begin_fill()
        tl.circle(5)
        # if p.has_interesting_size():
        if p.has_interesting_size():
            tl.end_fill()
        tl.penup()

    for p_i in range(len(poi)):
        p = poi[p_i]
        (x, y) = robot.grobot.get_xy(p.average, p.get_centered_angle())
        tl.goto((mm2px(x)+cx, mm2px(y)-5+cy))
        tl.color('green', 'green')
        tl.pendown()
        tl.begin_fill()
        tl.circle(5)
        tl.end_fill()
        tl.penup()

    tl.color('black', 'black')
    tl.pensize(1)
    tl.goto(offset)




def init_turtle():
    # Setup window size
    s = turtle.Screen()
    s.setup(WINDOW_WIDTH, WINDOW_HEIGHT)
    s.bgcolor("darkgrey")

    # Turtle config
    turtle.title("poklidar-simulator")
    turtle.tracer(0, 0)
    turtle.delay(0)
    t = turtle.Turtle(visible=False)
    t.penup()
    t.speed(0)
    return t

def update_graphics(tl, obstacles, robots):
    tl.clear()
    tl.goto(TABLE_CENTER)
    draw_table(tl)
    draw_obstacles(tl, obstacles)
    for r in robots:
        draw_robot(tl, r)

    tl.goto(500, 0)
    rad_data = robots[0].lidar_data
    draw_raw_lidar(tl, rad_data, robots[0].lidar_results, robots[0])

    turtle.update()
