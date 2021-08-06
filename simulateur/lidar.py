import math
import random
import sys

from shapely.geometry.polygon import LineString, Point
from shapely.geometry import MultiLineString
from shapely.ops import nearest_points

from config import *

# seed = random.randrange(sys.maxsize)
# rng = random.Random(seed)
# print("Seed was:", seed)

random.seed(56467)


class Jean_Michel:
    def __init__(self, x0, y0, x1, y1, s = 20):
        self.x0 = x0
        self.y0 = y0
        self.x1 = x1
        self.y1 = y1
        self.x  = x0
        self.y  = y0

        self.s  = s
        self.dx = s
        self.dy = s
    def simulate(self):
        self.x+=self.dx
        self.y+=self.dy

        if self.x < self.x0:
            self.dx = self.s
        elif self.x > self.x1:
            self.dx = -self.s
        elif self.y < self.y0:
            self.dy = self.s
        elif self.y > self.y1:
            self.dy = -self.s



def get_circle_obstacle(c, r):
    if ENABLE_FAST_GRAPHICS:
        (x, y) = c
        points = []
        EDGES = 4
        for i in range(EDGES+1):
            angle = (2*i+1)*(math.pi/EDGES)
            points+=[(1.2*r*math.cos(angle)+x, math.sqrt(2)*r*math.sin(angle)+y)]
        return LineString(points)
    else:
        return Point(c).buffer(r).boundary

def get_jean_michel(x, y, width, height):
    jean_michel = LineString([
        (x-width/2, y-height/2),
        (x+width/2, y-height/2),
        (x+width/2, y+height/2),
        (x-width/2, y+height/2),
        (x-width/2, y-height/2)
    ])
    return jean_michel

# Generate a list of obstacles
# Each obstacle is a LineString
def get_basic_obstacles():
    obstacles = []
    # Mat central. /!\ Dimensions non normees par le reglement /!\
    obstacles += [LineString([
        (-100, 1000+22+200),
        (-11, 1000+22+200),
        (-11, 1000+22),
        (11, 1000+22),
        (11, 1000+22+200),
        (100, 1000+22+200),
        (100, 1000+22+200+22),
        (-100, 1000+22+200+22),
        (-100, 1000+22+200)
    ])]

    # Balises equipe cote droit
    # Diametre de 9cm pour avoir 1cm de marge
    if (START_ON_THE_RIGHT and WE_HAVE_BEACONS) or (not START_ON_THE_RIGHT and ENNEMY_HAS_BEACONS):
        obstacles += [get_circle_obstacle((-1500-22-50, +1000-50), 45)]
        obstacles += [get_circle_obstacle((-1500-22-50, -1000+50), 45)]
        obstacles += [get_circle_obstacle((+1500+22+50, 0), 45)]

    # Balises equipe cote gauche
    # Diametre de 9cm pour avoir 1cm de marge
    if (not START_ON_THE_RIGHT and WE_HAVE_BEACONS) or (START_ON_THE_RIGHT and ENNEMY_HAS_BEACONS):
        obstacles += [get_circle_obstacle((+1500+22+50, +1000-50), 45)]
        obstacles += [get_circle_obstacle((+1500+22+50, -1000+50), 45)]
        obstacles += [get_circle_obstacle((-1500-22-50, 0), 45)]

    # Les Jean-Michel casse-couilles
    if JEAN_MICHEL_CASSE_COUILLES:
        # 2 arbitres
        obstacles += [get_jean_michel(-1000, +1000+400, JEAN_MICHEL_WIDTH, JEAN_MICHEL_WIDTH)]
        obstacles += [get_jean_michel(+1000, +1000+400, JEAN_MICHEL_WIDTH, JEAN_MICHEL_WIDTH)]
        # 2 Jean-Michel cote droit
        obstacles += [get_jean_michel(+1500+400, +600, JEAN_MICHEL_WIDTH, JEAN_MICHEL_WIDTH)]
        obstacles += [get_jean_michel(+1500+400, -600, JEAN_MICHEL_WIDTH, JEAN_MICHEL_WIDTH)]
        # 2 Jean-Michel cote gauche
        obstacles += [get_jean_michel(-1500-400, +600, JEAN_MICHEL_WIDTH, JEAN_MICHEL_WIDTH)]
        obstacles += [get_jean_michel(-1500-400, -600, JEAN_MICHEL_WIDTH, JEAN_MICHEL_WIDTH)]

    return obstacles

def get_dynamic_obstacles(robots, jean_michels):
    obstacles = []
    for r in robots:
        obstacles+=[get_circle_obstacle(r.get_center(), 40)]
    for j in jean_michels:
        obstacles+=[get_jean_michel(j.x, j.y, JEAN_MICHEL_WIDTH, JEAN_MICHEL_WIDTH)]
    return obstacles

# x, y, r: position and rotation (degrees) of lidar on the table
# obstacles: obstacle list from get_obstacles()
# Returns (rad_data, cart_data)
# rad_data is a list of distance in mm for each sampled angle
# cart_data is a list of carthesian points on the table
def get_lidar_data(x, y, r, obstacles, i0 = 0, i1 = LIDAR_POINTS_PER_TURN):
    all_obstacles = MultiLineString(obstacles)
    rad_data = []
    for i in range(i0, i1):
        angle = r*math.pi/180 # Initial angle due to robot rotation
        if LIDAR_ROTATE_CLOCKWISE:
            angle += -2*math.pi*i/LIDAR_POINTS_PER_TURN
        else:
            angle += 2*math.pi*i/LIDAR_POINTS_PER_TURN
        angle_error = (2*math.pi/LIDAR_POINTS_PER_TURN)*LIDAR_ANGLE_ERROR*(2*random.random()-1)
        angle+=angle_error
        # Line of sight begins 100mm away from the center
        # to avoid detecting robot itself
        (start_x, start_y) = (x+100*math.cos(angle), y+100*math.sin(angle))
        # Point at the "infinite" where the lidar aims
        (aim_x, aim_y) = (x+5000*math.cos(angle), y+5000*math.sin(angle))
        line_of_sight = LineString([(start_x, start_y), (aim_x, aim_y)])

        points = line_of_sight.intersection(all_obstacles)
        if not points.is_empty:
            p1, p2 = nearest_points(Point(x, y), points)
            dist = Point(x, y).distance(p2)
            rad_data+=[dist + (2*random.random()-1)*LIDAR_DISTANCE_ERROR*dist]
        else:
            rad_data+=[0]
    return rad_data
