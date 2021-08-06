import math

app_pi = math.pi

(sin_counter, cos_counter, tan_counter, atan_counter, atan2_counter) = (0, 0, 0, 0, 0)

def app_reset_counters():
    (sin_counter, cos_counter, tan_counter, atan_counter, atan2_counter) = (0, 0, 0, 0, 0)

def app_get_counters():
    return (sin_counter, cos_counter, tan_counter, atan_counter, atan2_counter)

def app_sin(a):
    global sin_counter
    sin_counter+=1
    return math.sin(a)
def app_cos(a):
    global cos_counter
    cos_counter+=1
    return math.cos(a)
def app_tan(a):
    global tan_counter
    tan_counter+=1
    return math.tan(a)
def app_atan(a):
    global atan_counter
    atan_counter+=1
    return math.atan(a)
def app_atan2(a, b):
    global atan2_counter
    atan2_counter+=1
    return math.atan2(a, b)
