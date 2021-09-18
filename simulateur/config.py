# Match config
START_ON_THE_RIGHT = True
WE_HAVE_BEACONS    = True
ENNEMY_HAS_BEACONS = True

# Add obstacles to simulate people around the table
JEAN_MICHEL_CASSE_COUILLES = True
# People are squares with the size defined bellow
JEAN_MICHEL_WIDTH = 90

ENABLE_FAST_GRAPHICS = True

# Maximum speed on X or Y axis (in mm/s)
ROBOT_MAX_HSPEED = 1000
# Maximum rotational speed (in degree per second)
ROBOT_MAX_RSPEED = 360

ROBOT_NERF = 1


# Hardware config
LIDAR_FREQUENCY = 10
LIDAR_POINTS_PER_TURN  = 400
LIDAR_ROTATE_CLOCKWISE = True

LIDAR_ANGLE_ERROR = 0.5
LIDAR_DISTANCE_ERROR = 0.02 # 2% resolution

LIDAR_ANGLE_PER_STEP = 400
LIDAR_PACKET_SIZE = 6

# Graphic constant
PIXEL_PER_METER = 200

TABLE_WIDTH  = 3000
TABLE_HEIGHT = 2000

WINDOW_WIDTH  = 1800
WINDOW_HEIGHT = 900

TABLE_CENTER = (-350, 0)


# Do not edit those lines
ROBOT_MAX_HSPEED_PT = ROBOT_MAX_HSPEED/LIDAR_FREQUENCY
ROBOT_MAX_RSPEED_PT = ROBOT_MAX_RSPEED/LIDAR_FREQUENCY
