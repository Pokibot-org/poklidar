import math

from config import *

# Delta filter threshold
DELTA_FILTER = 150

A_DISTANCE = 3284
B_DISTANCE = 1900
AB_ANGLE   = math.atan((1000-50)/(3000+44+100))



class PTT:
    def __init__(self):
        self.x = 0
        self.y = 0

# Guessed robot
class GRobot:
    def __init__(self, x, y, r):
        self.x = x # Position X
        self.y = y # Position Y
        self.r = r # Rotation R

        self.ptt_list = []

    def get_beacons(self):
        beacons = []
        if START_ON_THE_RIGHT:
            beacons += [(-1500-22-50, +1000-50)]
            beacons += [(-1500-22-50, -1000+50)]
            beacons += [(+1500+22+50, 0)]
        else:
            beacons += [(+1500+22+50, +1000-50)]
            beacons += [(+1500+22+50, -1000+50)]
            beacons += [(-1500-22-50, 0)]
        return beacons
    def get_xy(self, d, r):
        angle = r+self.r*math.pi/180
        return (self.x+d*math.cos(angle), self.y+d*math.sin(angle))


# POI = Point Of Interest
class POI:
    def __init__(self, i0, i1, average, size):
        self.i0 = i0 # Discrete start angle
        self.i1 = i1 # Discrete stop angle
        self.average = average # Average distance of the points in the POI
        self.size = size # Estimated size of the POI
    def get_all(self):
        return (self.i0, self.i1, self.average, self.size)
    def has_interesting_size(self):
        return self.size > 30 and self.size < 150
    def get_centered_angle(self):
        if LIDAR_ROTATE_CLOCKWISE:
            return -(self.i1-1+self.i0)*math.pi/LIDAR_POINTS_PER_TURN
        else:
            return (self.i1-1+self.i0)*math.pi/LIDAR_POINTS_PER_TURN


# Calculate difference between each measured distance by the lidar
def get_delta(distance):
    delta = [0]*len(distance)
    for i in range(len(distance)):
        delta[i] = distance[i]-distance[i-1]
    return delta
# Filter with a threshold
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
    # TODO: case when delta[i] always == 0
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
                # Don't know If should use a tan() or a sin(),
                # but both produces not so good results
                # This is possibly due to the distance/angle error
                size = 2 * average * math.tan( ((i1-i0))*math.pi/LIDAR_POINTS_PER_TURN )
                # Register POI
                p = POI(i0, i1, average, size)
                poi+=[p]
            i0 = i1
            average = 0
        average+=lidar_data[j]
        i1 = i1+1

    return poi

# Computes a1-a0
def angle_dist(a0, a1):
    a = a1-a0
    while a < 0:
        a+=LIDAR_POINTS_PER_TURN
    while a >= LIDAR_POINTS_PER_TURN:
        a-=LIDAR_POINTS_PER_TURN
    return a

# Search POIs nexto one POI
def who_is_near_me(poi, my_index):
    DIST_MAX = 300
    my_poi = poi[my_index]
    my_angle = (my_poi.i0+my_poi.i1-1)/2
    d = my_poi.average

    # Max angle to search in
    max_angle = 2*math.atan((DIST_MAX/2)/d) # Radians
    max_angle = max_angle*LIDAR_POINTS_PER_TURN/(2*math.pi) # Lidar ticks

    result = []

    i = (my_index+1)%len(poi)
    while angle_dist(my_angle, poi[i].i0) < max_angle:
        p = poi[i]
        if abs(p.average-my_poi.average) < DIST_MAX/2:
            result+=[i]
        i = (i+1)%len(poi)

    i = (my_index-1+len(poi))%len(poi)
    while angle_dist(poi[i].i1, my_angle) < max_angle:
        p = poi[i]
        if abs(p.average-my_poi.average) < DIST_MAX/2:
            result+=[i]
        i = (i-1+len(poi))%len(poi)


    return result

def get_interesting_pois(poi):
    r = []
    for p in poi:
        if p.has_interesting_size():
            r+=[p]
    return r

# Return squared distance between 2 points in polar coordinates
def get_rdist2(d0, r0, d1, r1):
    return d0*d0+d1*d1-2*d0*d1*math.cos(r1-r0)

def get_rdist(d0, r0, d1, r1):
    return math.sqrt(d0*d0+d1*d1-2*d0*d1*math.cos(r1-r0))

def get_dist(a, b):
    (x0, y0) = a
    (x1, y1) = b
    return math.sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0))

def get_dist_table(poi):
    table = [[0]*len(poi) for i in range(len(poi))]
    a_table = []
    b_table = []
    for i in range(len(poi)):
        for j in range(0, i, 1):
            table[i][j] = get_rdist(poi[i].average, poi[i].get_centered_angle(), poi[j].average, poi[j].get_centered_angle())
            if abs(table[i][j]-A_DISTANCE) < 300:
                a_table+=[(i, j)]
            if abs(table[i][j]-B_DISTANCE) < 300:
                b_table+=[(i, j)]
    return (table, a_table, b_table)


def detect_beacons(poi, grobot):
    previous_beacons = grobot.get_beacons()
    candidates = [[], [], []]
    for bi in range(3):
        for i in range(len(poi)):
            p = poi[i]
            (px, py) = grobot.get_xy(p.average, p.get_centered_angle())
            d = get_dist((px, py), (previous_beacons[bi]))
            if d < 500:
                candidates[bi]+=[i]
        if len(candidates[bi]) == 0:
            candidates[bi]+=[-1]
    return candidates

def check_solution(poi, solution):
    (s0, s1, s2) = solution
    (d01, d02, d12) = (B_DISTANCE, A_DISTANCE, A_DISTANCE)
    if s0 != -1 and s1 != -1:
        d01 = get_rdist(poi[s0].average, poi[s0].get_centered_angle(), poi[s1].average, poi[s1].get_centered_angle())
    if s0 != -1 and s2 != -1:
        d02 = get_rdist(poi[s0].average, poi[s0].get_centered_angle(), poi[s2].average, poi[s2].get_centered_angle())
    if s1 != -1 and s2 != -1:
        d12 = get_rdist(poi[s1].average, poi[s1].get_centered_angle(), poi[s2].average, poi[s2].get_centered_angle())
    score = abs(d01-B_DISTANCE)**2 + abs(d02-A_DISTANCE)**2 + abs(d12-A_DISTANCE)**2
    return score

def find_solution_aux(poi, candidates):
    best_solution = (-1, -1, -1)
    best_score = (150**2)*3
    for i0 in range(len(candidates[0])):
        for i1 in range(len(candidates[1])):
            for i2 in range(len(candidates[2])):
                solution = (candidates[0][i0], candidates[1][i1], candidates[2][i2])
                score = check_solution(poi, solution)
                # print(score)
                if score < best_score:
                    best_score = score
                    best_solution = solution
    return (best_solution, best_score)

def find_solution(poi, candidates):
    (solution, score) = find_solution_aux(poi, candidates)

    if solution == (-1, -1, -1):
        best_solution = (-1, -1, -1)
        best_score = (150**2)*3
        for i in range(3):
            new_candidates = [l for l in candidates]
            new_candidates[i] = [-1]
            (solution, score) = find_solution_aux(poi, new_candidates)
            if score < best_score:
                best_score = score
                best_solution = solution
        solution = best_solution
    return solution


def get_total_diff(data0, data1, s1):
    total = 0
    for i in range(LIDAR_POINTS_PER_TURN):
        total+=abs(data0[i]-data1[(i+s1)%LIDAR_POINTS_PER_TURN])
    return total

def pre_find_angle(data0, data1):
    best_i = 0
    best_t = -1
    for i in range(LIDAR_POINTS_PER_TURN):
        t = get_total_diff(data0, data1, i)
        if best_t == -1 or best_t > t:
            best_t = t
            best_i = i
    return best_i

def solve_angle(d0, r0, d1, r1, dopp):
    # https://math.stackexchange.com/questions/1365622/adding-two-polar-vectors
    # ϕ=ϕ1+arctan2(r2sin(ϕ2−ϕ1),r1+r2cos(ϕ2−ϕ1))
    # d1 is inverted to substract instead of adding
    return r0+math.atan2(-d1*math.sin(r1-r0), d0-d1*math.cos(r1-r0))

def apply_solution(poi, solution, grobot):
    ref_angle = grobot.r*math.pi/180
    (s0, s1, s2) = solution
    (a01, a02, a12) = (-1, -1, -1)
    angle_sum = 0
    pond_sum = 0
    if s0 != -1 and s1 != -1:
        a01 = solve_angle(poi[s1].average, poi[s1].get_centered_angle(), poi[s0].average, poi[s0].get_centered_angle(), B_DISTANCE)
        a01 = a01 + math.pi/2
        while a01-ref_angle > math.pi:
            a01-=2*math.pi
        while a01-ref_angle <= -math.pi:
            a01+=2*math.pi
        pond = 5000-(poi[s0].average+poi[s1].average)/2
        angle_sum+=pond*a01
        pond_sum+=pond
    if s0 != -1 and s2 != -1:
        a02 = solve_angle(poi[s0].average, poi[s0].get_centered_angle(), poi[s2].average, poi[s2].get_centered_angle(), A_DISTANCE)
        a02 = a02 - (math.pi/2+math.pi/2-AB_ANGLE)
        while a02-ref_angle > math.pi:
            a02-=2*math.pi
        while a02-ref_angle <= -math.pi:
            a02+=2*math.pi
        pond = 5000-(poi[s0].average+poi[s2].average)/2
        angle_sum+=pond*a02
        pond_sum+=pond
    if s1 != -1 and s2 != -1:
        a12 = solve_angle(poi[s1].average, poi[s1].get_centered_angle(), poi[s2].average, poi[s2].get_centered_angle(), A_DISTANCE)
        a12 = a12 - (math.pi+AB_ANGLE)
        while a12-ref_angle > math.pi:
            a12-=2*math.pi
        while a12-ref_angle <= -math.pi:
            a12+=2*math.pi
        pond = 5000-(poi[s1].average+poi[s2].average)/2
        angle_sum+=pond*a12
        pond_sum+=pond

    if pond_sum != 0:
        r = -angle_sum / pond_sum
    else:
        print("WARNING: Angle was not found")
        r = grobot.r*math.pi/180
    while(r >= 2*math.pi):r-=2*math.pi
    while(r < 0):r+=2*math.pi

    # print("Angle found", s0, s1, s2)
    # print(a01*180/math.pi, a02*180/math.pi, a12*180/math.pi, a01, a02, a12)

    # We finally got the angle !
    # Now, we just need to get the position

    beacons = grobot.get_beacons()
    x_sum = 0
    y_sum = 0
    pond_sum = 0
    for i in range(len(solution)):
        if solution[i] != -1:
            p = poi[solution[i]]
            a = -math.pi-p.get_centered_angle()-r
            dx = p.average*math.cos(a)
            dy = p.average*math.sin(a)
            (bx, by) = beacons[i]
            (x, y) = (bx+dx, by-dy)
            pond = 5000-p.average
            x_sum+=pond*x
            y_sum+=pond*y
            pond_sum+=pond
    (x, y) = (grobot.x, grobot.y)
    if pond_sum != 0:
        x = x_sum/pond_sum
        y = y_sum/pond_sum
    else:
        print("WARNING: Position was not found")
    r = r*180/math.pi
    # print("Found:", x, y, r)

    grobot.x = x
    grobot.y = y
    grobot.r = r




# Main function
# lidar_data is a list of distances measured by the lidar
def solve_lidar(lidar_data, prev_lidar_data, grobot):

    pre_angle = pre_find_angle(lidar_data, prev_lidar_data)*360/LIDAR_POINTS_PER_TURN
    print("PRE_ANGLE", pre_angle)
    # if pre_angle > 330 or pre_angle < 318:
    #     print("AAAAAAAAAAAAAAAAAAAAAAAAAA", pre_angle)
    if LIDAR_ROTATE_CLOCKWISE:
        grobot.r = (grobot.r-pre_angle+360)%360
    else:
        grobot.r = (grobot.r+pre_angle+360)%360


    delta = get_delta(lidar_data)
    delta = filter_delta(delta)
    poi = get_poi(lidar_data, delta)

    voisins = []
    for i in range(len(poi)):
        p = poi[i]
        # if p.i1 < len(lidar_data):
        #     print(p.i0, p.i1, lidar_data[p.i0-1:p.i1+1])
        # else:
        #     print(p.i0, p.i1, lidar_data[p.i0-1:]+lidar_data[:p.i1-len(lidar_data)+1])
        if p.has_interesting_size():
            r = who_is_near_me(poi, i)
            # print(i, r)
            if len(r) != 0:
                voisins+=r+[i]

    # interesting_pois = get_interesting_pois(poi)
    # (table, a, b) = get_dist_table(interesting_pois)
    # print('\n'.join([str(i) for i in table]))
    # print(a)
    # print(b)

    poi_algo_t = []
    for p in poi:
        if p.has_interesting_size():
            poi_algo_t += [p]

    candidates = detect_beacons(poi_algo_t, grobot)
    solution = find_solution(poi_algo_t, candidates)
    print(candidates)
    print(solution)

    apply_solution(poi_algo_t, solution, grobot)


    return (delta, poi, voisins)
