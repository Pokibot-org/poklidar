import math
from random import random

from shapely.geometry.polygon import LineString, Point
from shapely.geometry import MultiLineString
from shapely.ops import nearest_points

from config import *

# Generate a list of obstacles
# Each obstacle is a LineString
def get_basic_obstacles():
    obstacles = []
    # Mat central. /!\ Dimensions non normees par le reglement /!\
    obstacles += [LineString([(-100, 1000+22+200), (-11, 1000+22+200), (-11, 1000+22), (11, 1000+22), (11, 1000+22+200), (100, 1000+22+200), (100, 1000+22+200+22), (-100, 1000+22+200+22), (-100, 1000+22+200)])]

    # Balises equipe cote droit
    # Diametre de 9cm pour avoir 1cm de marge
    if (START_ON_THE_RIGHT and WE_HAVE_BEACONS) or (not START_ON_THE_RIGHT and ENNEMY_HAS_BEACONS):
        obstacles += [Point(-1500-22-50, +1000-50).buffer(45).boundary]
        obstacles += [Point(-1500-22-50, -1000+50).buffer(45).boundary]
        obstacles += [Point(+1500+22+50, 0).buffer(45).boundary]

    # Balises equipe cote gauche
    # Diametre de 9cm pour avoir 1cm de marge
    if (not START_ON_THE_RIGHT and WE_HAVE_BEACONS) or (START_ON_THE_RIGHT and ENNEMY_HAS_BEACONS):
        obstacles += [Point(+1500+22+50, +1000-50).buffer(45).boundary]
        obstacles += [Point(+1500+22+50, -1000+50).buffer(45).boundary]
        obstacles += [Point(-1500-22-50, 0).buffer(45).boundary]

    # Les Jean-Michel casse-couilles
    if JEAN_MICHEL_CASSE_COUILLES:
        # 2 arbitres
        obstacles += [LineString([(-1000, +1000+400), (-1000, +1000+400+JEAN_MICHEL_WIDTH), (-1000-JEAN_MICHEL_WIDTH, +1000+400+JEAN_MICHEL_WIDTH), (-1000-JEAN_MICHEL_WIDTH, +1000+400), (-1000, +1000+400)])]
        obstacles += [LineString([(+1000, +1000+400), (+1000, +1000+400+JEAN_MICHEL_WIDTH), (+1000+JEAN_MICHEL_WIDTH, +1000+400+JEAN_MICHEL_WIDTH), (+1000+JEAN_MICHEL_WIDTH, +1000+400), (+1000, +1000+400)])]
        # 2 Jean-Michel cote droit
        obstacles += [LineString([(+1500+400, +600), (+1500+400, +600+JEAN_MICHEL_WIDTH), (+1500+400+300, +600+JEAN_MICHEL_WIDTH), (+1500+400+300, +600), (+1500+400, +600)])]
        obstacles += [LineString([(+1500+400, -600), (+1500+400, -600-JEAN_MICHEL_WIDTH), (+1500+400+300, -600-JEAN_MICHEL_WIDTH), (+1500+400+300, -600), (+1500+400, -600)])]
        # 2 Jean-Michel cote gauche
        obstacles += [LineString([(-1500-400, +600), (-1500-400, +600+JEAN_MICHEL_WIDTH), (-1500-400-300, +600+JEAN_MICHEL_WIDTH), (-1500-400-300, +600), (-1500-400, +600)])]
        obstacles += [LineString([(-1500-400, -600), (-1500-400, -600-JEAN_MICHEL_WIDTH), (-1500-400-300, -600-JEAN_MICHEL_WIDTH), (-1500-400-300, -600), (-1500-400, -600)])]

    return obstacles

def get_dynamic_obstacles(robots):
    obstacles = []
    for r in robots:
        obstacles+=[Point(r.get_center()).buffer(40).boundary]
    return obstacles

# x, y, r: position and rotation (degrees) of lidar on the table
# obstacles: obstacle list from get_obstacles()
# Returns (rad_data, cart_data)
# rad_data is a list of distance in mm for each sampled angle
# cart_data is a list of carthesian points on the table
def get_lidar_data(x, y, r, obstacles):
    all_obstacles = MultiLineString(obstacles)
    cart_data = []
    rad_data = []
    for i in range(LIDAR_POINTS_PER_TURN):
        angle = r*math.pi/180 # Initial angle due to robot rotation
        if LIDAR_ROTATE_CLOCKWISE:
            angle += -2*math.pi*i/LIDAR_POINTS_PER_TURN
        else:
            angle += 2*math.pi*i/LIDAR_POINTS_PER_TURN
        angle_error = (2*math.pi/LIDAR_POINTS_PER_TURN)*LIDAR_ANGLE_ERROR*(2*random()*1)
        angle+=angle_error
        # Line of sight begins 50mm away from the center
        # to avoid detecting robot itself
        (start_x, start_y) = (x+50*math.cos(angle), y+50*math.sin(angle))
        # Point at the "infinite" where the lidar aims
        (aim_x, aim_y) = (5000*math.cos(angle), 5000*math.sin(angle))
        line_of_sight = LineString([(start_x, start_y), (aim_x, aim_y)])

        points = line_of_sight.intersection(all_obstacles)
        if not points.is_empty:
            p1, p2 = nearest_points(Point(x, y), points)
            cart_data+=[p2.coords[0]]
            dist = Point(x, y).distance(p2)
            rad_data+=[dist + (2*random()*1)*LIDAR_DISTANCE_ERROR*dist]
        else:
            cart_data+=[(x, y)]
            rad_data+=[0]
    return (rad_data, cart_data)
