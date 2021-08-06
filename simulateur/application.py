import math

from config import *
from trigo import *

# Delta filter threshold
DELTA_FILTER = 150

A_DISTANCE = 3284
B_DISTANCE = 1900
AB_ANGLE   = app_atan((1000-50)/(3000+44+100))



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

        self.too_fast_errors = 0

        self.lost_flag = False

    def get_xy(self, d, r):
        angle = r+self.r*app_pi/180
        return (self.x+d*app_cos(angle), self.y+d*app_sin(angle))



def get_beacons():
    beacons = []
    if START_ON_THE_RIGHT:
        beacons += [(-1500-22-50, +1000-50)]
        beacons += [(-1500-22-50, -1000+50)]
        beacons += [(+1500+22+50, 0)]
    else:
        beacons += [(+1500+22+50, -1000+50)]
        beacons += [(+1500+22+50, +1000-50)]
        beacons += [(-1500-22-50, 0)]
    return beacons

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
        return self.size > 10 and self.size < 160
    def get_centered_angle(self):
        if LIDAR_ROTATE_CLOCKWISE:
            return -(self.i1-1+self.i0)*app_pi/LIDAR_POINTS_PER_TURN
        else:
            return (self.i1-1+self.i0)*app_pi/LIDAR_POINTS_PER_TURN


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
                size = 2 * average * app_tan( ((i1-i0))*app_pi/LIDAR_POINTS_PER_TURN )
                # Register POI
                p = POI(i0, i1, average+35, size)
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

# Return squared distance between 2 points in polar coordinates
def get_rdist2(d0, r0, d1, r1):
    return d0*d0+d1*d1-2*d0*d1*app_cos(r1-r0)

def get_rdist(d0, r0, d1, r1):
    return math.sqrt(d0*d0+d1*d1-2*d0*d1*app_cos(r1-r0))

def get_dist(a, b):
    (x0, y0) = a
    (x1, y1) = b
    return math.sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0))


def detect_beacons(poi, grobot):
    previous_beacons = get_beacons()
    candidates = [[-1], [-1], [-1]]
    dist = [[200], [200], [200]]
    for bi in range(3):
        for i in range(len(poi)):
            p = poi[i]
            (px, py) = grobot.get_xy(p.average, p.get_centered_angle())
            d = get_dist((px, py), (previous_beacons[bi]))
            if d < 500:
                candidates[bi]+=[i]
                dist[bi]+=[d]
    return (candidates, dist)

def get_solution_score(poi, solution, dist):
    (s0, s1, s2) = solution
    (dist0, dist1, dist2) = dist
    (d01, d02, d12) = (B_DISTANCE+90, A_DISTANCE+90, A_DISTANCE+90)
    if s0 != -1 and s1 != -1:
        d01 = get_rdist(poi[s0].average, poi[s0].get_centered_angle(), poi[s1].average, poi[s1].get_centered_angle())
    if s0 != -1 and s2 != -1:
        d02 = get_rdist(poi[s0].average, poi[s0].get_centered_angle(), poi[s2].average, poi[s2].get_centered_angle())
    if s1 != -1 and s2 != -1:
        d12 = get_rdist(poi[s1].average, poi[s1].get_centered_angle(), poi[s2].average, poi[s2].get_centered_angle())
    score0 = abs(d01-B_DISTANCE)**2 + abs(d02-A_DISTANCE)**2 + abs(d12-A_DISTANCE)**2
    score1 = dist0**2 + dist1**2 + dist2**2
    return score0 + score1/100

def sort_solution(poi, candidates, dist):
    ranked_solutions = []
    for i0 in range(len(candidates[0])):
        for i1 in range(len(candidates[1])):
            for i2 in range(len(candidates[2])):
                solution = (candidates[0][i0], candidates[1][i1], candidates[2][i2])
                score = get_solution_score(poi, solution, (dist[0][i0], dist[1][i1], dist[2][i2]))
                if score < 200000:
                    print(solution, score)
                    j = 0
                    while j < len(ranked_solutions) and ranked_solutions[j][1] < score:
                        j+=1
                    ranked_solutions = ranked_solutions[:j] + [[solution, score]] + ranked_solutions[j:]
                else:
                    print(solution, score, "(removed)")
    return ranked_solutions

def get_total_diff(data0, data1, s1):
    total = 0
    for i in range(LIDAR_POINTS_PER_TURN):
        total+=abs(data0[i]-data1[(i+s1)%LIDAR_POINTS_PER_TURN])
    return total

# COST
# 100 * 400 = 40k
def pre_find_angle2(data0, data1):

    SEARCH_FACTOR = int(360/(ROBOT_MAX_RSPEED_PT+0.2*ROBOT_MAX_RSPEED_PT))

    diff_map = [0]*((LIDAR_POINTS_PER_TURN//SEARCH_FACTOR)*2+1)
    best_i = 0
    best_t = -1
    for i in range(-LIDAR_POINTS_PER_TURN//SEARCH_FACTOR, LIDAR_POINTS_PER_TURN//SEARCH_FACTOR + 1, 1):
        t = get_total_diff(data0, data1, i%LIDAR_POINTS_PER_TURN)
        diff_map[i + LIDAR_POINTS_PER_TURN//SEARCH_FACTOR] = t

    mins_locaux = []
    for j in range(1, (LIDAR_POINTS_PER_TURN//SEARCH_FACTOR)*2, 1):
        if diff_map[j-1] > diff_map[j] and diff_map[j] < diff_map[j+1]:
            mins_locaux += [(diff_map[j], (j-LIDAR_POINTS_PER_TURN//SEARCH_FACTOR)*360/LIDAR_POINTS_PER_TURN)]

    return sorted(mins_locaux)

def solve_angle(d0, r0, d1, r1, dopp):
    # https://math.stackexchange.com/questions/1365622/adding-two-polar-vectors
    # ϕ=ϕ1+arctan2(r2sin(ϕ2−ϕ1),r1+r2cos(ϕ2−ϕ1))
    # d1 is inverted to substract instead of adding
    if START_ON_THE_RIGHT:
        return r0+app_atan2(-d1*app_sin(r1-r0), d0-d1*app_cos(r1-r0))
    else:
        return r1+app_atan2(-d0*app_sin(r0-r1), d1-d0*app_cos(r0-r1))

def apply_solution(poi, solution, grobot):
    ref_angle = grobot.r*app_pi/180
    (s0, s1, s2) = solution
    (a01, a02, a12) = (-1, -1, -1)
    angle_sum = 0
    pond_sum = 0
    if s0 != -1 and s1 != -1:
        a01 = solve_angle(poi[s1].average, poi[s1].get_centered_angle(), poi[s0].average, poi[s0].get_centered_angle(), B_DISTANCE)
        a01 = a01 + app_pi/2
        while a01-ref_angle > app_pi:
            a01-=2*app_pi
        while a01-ref_angle <= -app_pi:
            a01+=2*app_pi
        pond = 5000-(poi[s0].average+poi[s1].average)/2
        pond = (poi[s0].average+poi[s1].average)/2
        angle_sum+=pond*a01
        pond_sum+=pond
    if s0 != -1 and s2 != -1:
        a02 = solve_angle(poi[s0].average, poi[s0].get_centered_angle(), poi[s2].average, poi[s2].get_centered_angle(), A_DISTANCE)
        a02 = a02 - (app_pi/2+app_pi/2-AB_ANGLE)
        while a02-ref_angle > app_pi:
            a02-=2*app_pi
        while a02-ref_angle <= -app_pi:
            a02+=2*app_pi
        pond = 5000-(poi[s0].average+poi[s2].average)/2
        pond = (poi[s0].average+poi[s2].average)/2
        angle_sum+=pond*a02
        pond_sum+=pond
    if s1 != -1 and s2 != -1:
        a12 = solve_angle(poi[s1].average, poi[s1].get_centered_angle(), poi[s2].average, poi[s2].get_centered_angle(), A_DISTANCE)
        a12 = a12 - (app_pi+AB_ANGLE)
        while a12-ref_angle > app_pi:
            a12-=2*app_pi
        while a12-ref_angle <= -app_pi:
            a12+=2*app_pi
        pond = 5000-(poi[s1].average+poi[s2].average)/2
        pond = (poi[s1].average+poi[s2].average)/2
        angle_sum+=pond*a12
        pond_sum+=pond
    print("Angle:", a01*180/app_pi, a02*180/app_pi, a12*180/app_pi)

    if pond_sum != 0:
        r = -angle_sum / pond_sum
    else:
        print("WARNING: Angle was not found")
        r = grobot.r*app_pi/180
    while(r >= 2*app_pi):r-=2*app_pi
    while(r < 0):r+=2*app_pi

    # print("Angle found", s0, s1, s2)
    # print(a01*180/app_pi, a02*180/app_pi, a12*180/app_pi, a01, a02, a12)

    # We finally got the angle !
    # Now, we just need to get the position

    beacons = get_beacons()
    x_sum = 0
    y_sum = 0
    pond_sum = 0
    for i in range(len(solution)):
        if solution[i] != -1:
            p = poi[solution[i]]
            a = -app_pi-p.get_centered_angle()-r
            dx = p.average*app_cos(a)
            dy = p.average*app_sin(a)
            (bx, by) = beacons[i]
            (x, y) = (bx+dx, by-dy)
            pond = 5000-p.average
            x_sum+=pond*x
            y_sum+=pond*y
            pond_sum+=pond
            # print("Ponderation:", x, y, pond)
    (x, y) = (grobot.x, grobot.y)
    if pond_sum != 0:
        x = x_sum/pond_sum
        y = y_sum/pond_sum
    else:
        print("WARNING: Position was not found")
    r = r*180/app_pi
    # print("Found:", x, y, r)

    return (x, y, r)

def norm_angle_lidar_0(a):
    while(a > LIDAR_POINTS_PER_TURN/2):
        a-=LIDAR_POINTS_PER_TURN
    while(a <= -LIDAR_POINTS_PER_TURN/2):
        a+=LIDAR_POINTS_PER_TURN
    return a

def object_between(poi, d, r, target_size):
    # size = 2 * average * app_tan( ((i1-i0))*app_pi/LIDAR_POINTS_PER_TURN )
    dangle = app_atan((target_size/2)/d)*LIDAR_POINTS_PER_TURN/(2*app_pi)
    if dangle <= 2:
        return True
    angle  = r*LIDAR_POINTS_PER_TURN/(2*app_pi)
    for p in poi:
        j0 = norm_angle_lidar_0(p.i0-angle)
        j1 = norm_angle_lidar_0(p.i1-angle)
        if (abs(j0)<dangle or abs(j1)<dangle or (j0<0 and j1>0)) and p.average < d+150:
            return True
    return False

def check_solution(poi, poi_algo_t, solution, coords, grobot):
    (x, y, r) = coords
    (s0, s1, s2) = solution
    solution = [s0, s1, s2]

    # Check if the solution puts the robot on the table
    if abs(x) > 1600 or abs(y) > 1100:
        print("Robot is out of the table", x, y)
        return False

    # Check that the robot does not teleport on the table
    dr = grobot.oldr-r
    while(dr < -180):dr+=360
    while(dr >= 180):dr-=360
    dr2 = grobot.r-r
    while(dr2 < -180):dr2+=360
    while(dr2 >= 180):dr2-=360
    if  grobot.too_fast_errors < 7 and (
        abs(grobot.x-x) > 2*ROBOT_MAX_HSPEED_PT
        or abs(grobot.y-y) > 2*ROBOT_MAX_HSPEED_PT
        or abs(dr) > ROBOT_MAX_RSPEED_PT+0.2*ROBOT_MAX_RSPEED_PT
    ):
        print("Robot cannot move that fast", abs(grobot.x-x), abs(grobot.y-y), abs(dr))
        grobot.too_fast_errors += 1
        return False

    # Check beacons
    beacons = get_beacons()
    for i in range(3):
        if solution[i] == -1:
            # We did not detected this beacon.
            # Check if there is something between the robot
            # and the expected position of the beacon
            # If nothing is detected, we should see the beacon,
            # so the solution is not valid
            (bx, by) = beacons[i]
            ro = app_atan2(by-y, bx-x)-r*app_pi/180
            if LIDAR_ROTATE_CLOCKWISE:
                ro = -ro
            do = get_dist((x, y), (bx, by))
            if not object_between(poi, do, ro, 120):
                print("No object", i, do, ro*180/app_pi)
                return False
        else:
            # We detected the beacon.
            # We calculate the distance between the measured position
            # and the expected position of the beacon
            # If this error is greater than a percentage of the distance,
            # we consider the solution is invalid
            (bx, by) = beacons[i]
            p = poi_algo_t[solution[i]]
            angle = p.get_centered_angle() + r*app_pi/180
            (cx, cy) = (p.average*app_cos(angle)+x, p.average*app_sin(angle)+y)
            d = get_dist((bx, by), (cx, cy))
            if d > 90:
                print("Beacon too far away", i, d)
                return False

    # Check we see the central mast where it should be,
    # or that we see an obstructing object between the robot and the mast
    (bx, by) = (0, 1150)
    ro = app_atan2(by-y, bx-x)-r*app_pi/180
    if LIDAR_ROTATE_CLOCKWISE:
        ro = -ro
    do = get_dist((x, y), (bx, by))
    if not object_between(poi, do, ro, 400):
        print("No central mast found", do, ro*180/app_pi)
        return False

    # If all test are ok, then the solution is considered valid
    return True




# Main function
# lidar_data is a list of distances measured by the lidar
def solve_lidar(lidar_data, prev_lidar_data, grobot, solve = True):


    app_reset_counters()

    # Cost: 40k
    pre_angles = pre_find_angle2(lidar_data, prev_lidar_data)
    # print(pre_angles)

    grobot.oldr = grobot.r

    solution = None
    solution_found = False

    pre_anlge_index = 0

    while pre_anlge_index < len(pre_angles) and not solution_found:

        (_, pre_angle) = pre_angles[pre_anlge_index]
        print("PRE_ANGLE tried:", pre_angle)
        if LIDAR_ROTATE_CLOCKWISE:
            grobot.r = (grobot.oldr-pre_angle+360)%360
        else:
            grobot.r = (grobot.oldr+pre_angle+360)%360


        # Cost: 400
        delta = get_delta(lidar_data)
        delta = filter_delta(delta)
        # Cost: 30*TAN
        poi   = get_poi(lidar_data, delta)


        if not solve:
            break


        poi_algo_t = []
        for p in poi:
            if p.has_interesting_size():
                poi_algo_t += [p]

        # Cost: 3*20*COS = 60*COS
        (candidates, dist) = detect_beacons(poi_algo_t, grobot)
        # Cost: 4*4*4*3*COS = 192*COS
        ranked_solutions = sort_solution(poi_algo_t, candidates, dist)
        print(ranked_solutions)


        loop = True
        si = 0
        # Cost : x10
        # = 60*ATAN + 60*COS + 30*TAN
        while si < len(ranked_solutions) and loop:
            solution = ranked_solutions[si][0]
            print("Checking", solution)
            # Cost: 3*COS + 3*ATAN + 3*COS = 6*COS + 3*ATAN
            (x, y, r) = apply_solution(poi_algo_t, solution, grobot)
            # Cost: 3*TAN + 3*ATAN
            loop = not check_solution(poi, poi_algo_t, solution, (x, y, r), grobot)
            si+=1
        if si == len(ranked_solutions):
            solution = (-1, -1, -1)
            (x, y, r) = (grobot.x, grobot.y, grobot.r)
        if solution == (-1, -1, -1):
            print("WARNING: no solution found")
            print("Trying with another PRE_ANGLE...")
        else:
            solution_found = True

        pre_anlge_index = pre_anlge_index + 1

    print(solution)

    if solve and solution == (-1, -1, -1):
        print("C'est la full merde")
        input()

    if solve:
        grobot.x = x
        grobot.y = y
        grobot.r = r

        if grobot.too_fast_errors > 0:
            grobot.too_fast_errors-=1

    print("Operations:", app_get_counters())

    return (delta, poi)
