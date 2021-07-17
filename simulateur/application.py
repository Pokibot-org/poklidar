import math

from config import *

DELTA_FILTER = 100

class POI:
    def __init__(self, i0, i1, average, size):
        self.i0 = i0
        self.i1 = i1
        self.average = average
        self.size = size
    def get_all(self):
        return (self.i0, self.i1, self.average, self.size)
    def has_interesting_size(self):
        return self.size > 30 and self.size < 100



def get_delta(distance):
    delta = [0]*len(distance)
    for i in range(len(distance)):
        delta[i] = distance[i]-distance[i-1]
    return delta

def filter_delta(delta):
    d = [0]*len(delta)
    for i in range(len(delta)):
        if abs(delta[i]) < DELTA_FILTER:
            d[i] = 0
        else:
            d[i] = delta[i]
    return d

# Get Points Of Interest
def get_poi(lidar_data, delta):
    poi = []
    i_start = 0
    # TODO: case where delta[i] always == 0
    while delta[i_start] == 0:
        i_start+=1
    i0 = i_start
    average = lidar_data[i0]
    i1 = i_start+1
    while i1 != i_start+LIDAR_POINTS_PER_TURN+1:
        j = i1%LIDAR_POINTS_PER_TURN
        if delta[j] != 0:
            average = average/(i1-i0)
            # Remove POI with average bellow a certain value
            if average > 15:
                # Estimate object size
                size = 2 * average * math.tan( ((i1-1-i0))*math.pi/LIDAR_POINTS_PER_TURN )
                # Register POI
                p = POI(i0, i1, average, size)
                poi+=[p]
            i0 = i1
            average = 0
        average+=lidar_data[j]
        i1 = i1+1

    return poi

# def detect_group(poi):


def solve_lidar(lidar_data):
    delta = get_delta(lidar_data)
    delta = filter_delta(delta)
    poi = get_poi(lidar_data, delta)
    # print(len(poi), [d for (_, _, d) in poi])
    # print([s for (_, _, d, s) in poi])
    return (delta, poi)
