""" Process data locally """

import time
import math
import pprint
from simple_pid import PID
# from numpy import clip

Settings = {}
SetPoints = {}
InitialData = {}
CurrentData = {}
Diffs = {}
PIDS = {}
# initial, takeoff, climbing, sethead,
# setspeed
# level, descending, landing, stop
States = {}

PROGRAM = 'round'
RW_HEAD = 52
#PROGRAM = 'straight'
#PROGRAM = 'n_straight'
#PROGRAM = 'zig_zag'
#PROGRAM = 'turns'
#PROGRAM = 'landing'
#PROGRAM = 'n_setalt'
#PROGRAM = 'runway_center'
#PROGRAM = 'descend'

VERBOSE = True

def error_pause(_s,_t):
    """ Print message and sleep """
    print("ERROR: {}".format(_s))
    time.sleep(_t)

def init():
    # pylint: disable=too-many-statements
    # pylint: disable=too-many-branches
    """ Init data for local processing """
    Settings['takeoffspeed'] = 60.0
    Settings['prelanding_speed'] = 70.0
    Settings['targetspeed'] = 120.0
    Settings['landingalt'] = 40.0
    Settings['landing_speed'] = 40.0
    Settings['landingpitch'] = 7.0
    Settings['landingclimb'] = -0.3
    Settings['dropspeed_ground_alt'] = 4.0
    Settings['glissadealt'] = 450.0
    Settings['glissadespeed'] = 55.0
    Settings['engine_on_rpm'] = 100
    Settings['turnbank'] = 30
    Settings['turn_headingdelta'] = 3
    Settings['level_distancedelta'] = 40
    PIDS['rudder_runway'] = PID(0.01, 0.06, 0.01, setpoint=0)
    PIDS['rudder_runway'].output_limits = (-0.3, 0.3)
    PIDS['rudder_landing'] = PID(0.02, 0.003, 0.0, setpoint=0)
    PIDS['rudder_landing'].output_limits = (-0.5, 0.5)
    PIDS['rudder_flight'] = PID(0.03, 0.005, 0.0, setpoint=0)
    PIDS['rudder_flight'].output_limits = (-0.5, 0.5)
    PIDS['aileron_level'] = PID(0.04, 0.01, 0.01, setpoint=0)
    PIDS['aileron_level'].output_limits = (-0.4, 0.4)
    PIDS['aileron_turn'] = PID(0.01, 0.0, 0.0, setpoint=0)
    PIDS['aileron_turn'].output_limits = (-0.4, 0.4)
    PIDS['throttle'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['throttle'].output_limits = (0.005, 1.0)
    PIDS['elevator_climb_landing'] = PID(0.01, 0.008, 0.002, setpoint=0)
    PIDS['elevator_climb_landing'].output_limits = (-0.3, 0.1)
    PIDS['elevator_climb_flight'] = PID(0.008, 0.003, 0.008, setpoint=0)
    PIDS['elevator_climb_flight'].output_limits = (-0.2, 0.1)
    PIDS['elevator_pitch'] = PID(0.04, 0.005, 0.0, setpoint=0)
    PIDS['elevator_pitch'].output_limits = (-0.2, 0.1)
    States['current'] = 0
    States['program'] = []

    if PROGRAM == 'straight':
        States['program'].append({ 'name': 'initial' })
        States['program'].append({ 'name': 'setspeed', 'arg': 120 })
        States['program'].append({ 'name': 'setalt', 'arg': 150 })
        States['program'].append({ 'name': 'takeoff' })
        States['program'].append({ 'name': 'climbing' })
        States['program'].append({ 'name': 'level', 'arg': (0, 20000) })
        States['program'].append({ 'name': 'stop' })
        ################
    elif PROGRAM == 'landing':
        States['program'].append({ 'name': 'initial' })
        States['program'].append({ 'name': 'setspeed', 'arg': 70 })
        States['program'].append({ 'name': 'setalt', 'arg': 100 })
        States['program'].append({ 'name': 'takeoff' })
        States['program'].append({ 'name': 'climbing' })
        States['program'].append({ 'name': 'descending', 'arg': (0,2500) })
        States['program'].append({ 'name': 'landing' })
        States['program'].append({ 'name': 'stop' })
        ################
    elif PROGRAM == 'n_straight':
        States['program'].append({ 'name': 'initial' })
        States['program'].append({ 'name': 'setspeed', 'arg': 120 })
        States['program'].append({ 'name': 'setalt', 'arg': 150 })
        States['program'].append({ 'name': 'takeoff' })
        States['program'].append({ 'name': 'climbing' })
        for _n in range(20):
            States['program'].append({ 'name': 'level', 'arg': (0, 2000 + _n * 200) })
        States['program'].append({ 'name': 'stop' })
        ################
    elif PROGRAM == 'n_setalt':
        States['program'].append({ 'name': 'initial' })
        States['program'].append({ 'name': 'setspeed', 'arg': 60 })
        States['program'].append({ 'name': 'setalt', 'arg': 150 })
        States['program'].append({ 'name': 'takeoff' })
        States['program'].append({ 'name': 'climbing' })
        for _n in range(0, 20, 2):
            States['program'].append({ 'name': 'level', 'arg': (0, 2000 + _n * 1000) })
            States['program'].append({ 'name': 'setalt', 'arg': 300 })
            States['program'].append({ 'name': 'level', 'arg': (0, 2000 + (_n+1) * 1000) })
            States['program'].append({ 'name': 'setalt', 'arg': 150 })
        States['program'].append({ 'name': 'stop' })
        ################
    elif PROGRAM == 'zig_zag':
        States['program'].append({ 'name': 'initial' })
        States['program'].append({ 'name': 'setspeed', 'arg': 120 })
        States['program'].append({ 'name': 'setalt', 'arg': 150 })
        States['program'].append({ 'name': 'takeoff' })
        States['program'].append({ 'name': 'climbing' })
        States['program'].append({ 'name': 'level', 'arg': (0, 2000) })
        for _n in range(1,20,2):
            States['program'].append({ 'name': 'level', 'arg': (100, 2000 + _n * 1000) })
            States['program'].append({ 'name': 'level', 'arg': (-100, 2000 + (_n+1) * 1000) })
        States['program'].append({ 'name': 'stop' })
        ################
    elif PROGRAM == 'runway_center':
        States['program'].append({ 'name': 'initial' })
        States['program'].append({ 'name': 'setspeed', 'arg': 10 })
        States['program'].append({ 'name': 'takeoff' })
        States['program'].append({ 'name': 'stop' })
        ################
    elif PROGRAM == 'turns':
        States['program'].append({ 'name': 'initial' })
        States['program'].append({ 'name': 'setspeed', 'arg': 100 })
        States['program'].append({ 'name': 'setalt', 'arg': 150 })
        States['program'].append({ 'name': 'takeoff' })
        States['program'].append({ 'name': 'climbing' })
        States['program'].append({ 'name': 'sethead', 'arg': (200, 'left') })
        States['program'].append({ 'name': 'sethead', 'arg': (340, 'right') })
        States['program'].append({ 'name': 'sethead', 'arg': (200, 'left') })
        States['program'].append({ 'name': 'sethead', 'arg': (340, 'right') })
        States['program'].append({ 'name': 'sethead', 'arg': (200, 'left') })
        States['program'].append({ 'name': 'sethead', 'arg': (340, 'right') })
        States['program'].append({ 'name': 'stop' })
        ################
    elif PROGRAM == 'round':
        States['program'].append({ 'name': 'initial' })
        States['program'].append({ 'name': 'setspeed', 'arg': 120 })
        States['program'].append({ 'name': 'setalt', 'arg': 800 })
        States['program'].append({ 'name': 'takeoff' })
        States['program'].append({ 'name': 'climbing' })
        States['program'].append({ 'name': 'sethead', 'arg': ((RW_HEAD + 180)%360, 'left') })
        States['program'].append({ 'name': 'level', 'arg': (-700, -5000) })
        # Turn to the glissade, take off speed
        States['program'].append({ 'name': 'setspeed', 'arg': Settings['prelanding_speed'] })
        States['program'].append({ 'name': 'sethead', 'arg': (RW_HEAD, 'left') })
        # Lower to glissade start
        States['program'].append({ 'name': 'setalt', 'arg': Settings['glissadealt'] })
        States['program'].append({ 'name': 'level', 'arg': (0, -3000) })
        # Make flight level and take off speed to glissage's one
        States['program'].append({ 'name': 'sethead', 'arg': (RW_HEAD, '') })
        States['program'].append({ 'name': 'setspeed', 'arg': Settings['glissadespeed'] })
        States['program'].append({ 'name': 'level', 'arg': (0, -2000) })
        # Adjust heading
        States['program'].append({ 'name': 'sethead', 'arg': (RW_HEAD, '') })
        States['program'].append({ 'name': 'descending', 'arg': (0,0) })
        States['program'].append({ 'name': 'landing' })
        States['program'].append({ 'name': 'stop' })
        ################
    elif PROGRAM == 'descend':
        # Descend to 22R@PHNL    InitialData['heading'] = CurrentData['heading']
        head_22r = 232.8
        InitialData['heading'] = head_22r
        InitialData['altitude'] = 22.8
        InitialData['elevation'] = 19.57
        InitialData['ground_alt'] = InitialData['altitude'] - InitialData['elevation']
        InitialData['latitude'] = 21.329819
        InitialData['longitude'] = -157.907042
        SetPoints['altitude'] = None
        SetPoints['bank'] = 0.0
        SetPoints['heading'] = head_22r
        SetPoints['speed'] = None
        SetPoints['climb'] = None
        SetPoints['flaps'] = 0.0
        SetPoints['pitch'] = None

        States['program'].append({ 'name': 'setspeed', 'arg': 70 })
        States['program'].append({ 'name': 'setalt', 'arg': 600 })
        States['program'].append({ 'name': 'level', 'arg': (0, -3000) })
        States['program'].append({ 'name': 'sethead', 'arg': (head_22r, '') })
        States['program'].append({ 'name': 'setalt', 'arg': 450 })
        States['program'].append({ 'name': 'setspeed', 'arg': 60 })
        States['program'].append({ 'name': 'level', 'arg': (0, -2000) })
        States['program'].append({ 'name': 'sethead', 'arg': (head_22r, '') })
        States['program'].append({ 'name': 'descending', 'arg': (0, 0) })
        States['program'].append({ 'name': 'landing' })
        States['program'].append({ 'name': 'stop' })


    else:
        error_pause("Choose correct program", 1000)

def get_cur_state():
    """ Return current state """
    return States['program'][States['current']]['name']

def get_cur_arg():
    """ Return current state arg """
    return States['program'][States['current']]['arg']

def get_cur_flag():
    """ Return True if state flag is set """
    if 'flag' not in States['program'][States['current']]:
        return False
    return States['program'][States['current']]['flag']

def set_cur_flag(flag):
    """ Set state flag """
    States['program'][States['current']]['flag'] = flag

def next_state():
    """ Move to the next state """
    if get_cur_state() != 'stop':
        States['current'] += 1
    else:
        print("On the stop state, not advancing")

def bellshape(_x, flatness, limit = 1, zero = True):
    """ calculate bell-shaped coefficient.
        _x is a deviation
        when _x == 0 bellshape is limit
        when _x is large, bellshape approaches zero """
    return _shape(_x, flatness, limit = limit, zero = zero, twist = False)

def prop(_x0, _y0, _x1, _y1, _x, y_min = None, y_max = None):
    """ calculate proportional y for x """
    delta_x = _x1 - _x0
    delta_y = _y1 - _y0
    _y = _y0 + (_x - _x0) * delta_y / delta_x
    if y_min is not None and _y < y_min:
        _y = y_min
    elif y_max is not None and _y > y_max:
        _y = y_max
    return _y

def s_shape(_x, flatness, limit = 1, inverse = False):
    """ calculate s-shaped coefficient
        _x is a deviation
        when _x is positive and large, bellshape approaches limit
        when inverse is False, and -limit when inverse is True """
    return _shape(_x, flatness, limit = limit, inverse = inverse, twist = True)

def _shape(_x, flatness = 1, limit = 1, zero = True, inverse = False, twist = True):
    """ utility function """
    out = limit/((_x/flatness)**2 + 1)
    if zero:
        out = limit - out
    if twist:
        out = math.copysign(out,_x)
    if inverse:
        out = (-1) * out
    return out

def process_heading(heading_dev):
    """ Heading processing. Also set required bank angle """
    rudder = 0.0
    SetPoints['bank'] = 0.0

    if get_cur_state() in ('takeoff', 'stop'):
        rudder = PIDS['rudder_runway'](- heading_dev)

    if get_cur_state() in ('landing', 'descending'):
        rudder = PIDS['rudder_landing'](- heading_dev)

    if get_cur_state() in ('level', 'climbing', 'sethead'):
        # 1 at heading_dev == 0, near 0 at large heading_dev
        k_prop_rudder = bellshape(heading_dev, 20, zero = False)
        rudder = k_prop_rudder * PIDS['rudder_flight'](-heading_dev)

        # 0 at heading_dev == 0, near 1 at large heading_dev, with the same sign as heading_dev
        k_prop_aileron = s_shape(heading_dev, 8)
        SetPoints['bank'] = k_prop_aileron*Settings['turnbank']

    return rudder

def process_bank(bank_dev):
    """ Sample bank processing """
    aileron = 0.0
    if get_cur_state() == 'sethead':
        aileron = PIDS['aileron_turn'](bank_dev)
    elif get_cur_state() in ('level', 'climbing', 'descending', 'landing'):
        aileron = PIDS['aileron_level'](bank_dev)

    return aileron

def process_climb(climb_dev):
    """ Altitude processing on glissade """
    climb_pid = ''

    if get_cur_state() == 'landing':
        climb_pid = 'elevator_climb_landing'
    else:
        k_prop = prop(60, 0.008, 100, 0.004, CurrentData['speed'], y_min = 0.001)
        k_int = prop(60, 0.003, 100, 0.001, CurrentData['speed'], y_min = 0.0)
        k_der = prop(60, 0.008, 100, 0.004, CurrentData['speed'], y_min = 0.0)
        climb_pid = 'elevator_climb_flight'
        PIDS[climb_pid].tunings = (k_prop, k_int, k_der)

    elevator = PIDS[climb_pid](- climb_dev)
    return elevator

def process_speed(speed_dev):
    """ Flight speed processing """
    return PIDS['throttle'](speed_dev)

def get_xy_from_lat_lon(_lat, _lon):
    """ Calculate relative X and Y based on latitude and longitude """
    r_lat_local = math.radians(_lat - InitialData['latitude'])
    r_lon_local = math.radians(_lon - InitialData['longitude'])
    _x = 6371000 * r_lon_local * math.cos(math.radians(InitialData['latitude']))
    _y = 6371000 * r_lat_local
    return (_x, _y)

def get_lat_lon_from_xy(_x, _y):
    """ Calculate lat and lon from X and Y """
    r_lat_local = _y / 6371000
    r_lon_local = _x / 6371000 / math.cos(r_lat_local + math.radians(InitialData['latitude']))
    lat = math.degrees(r_lat_local) + InitialData['latitude']
    lon = math.degrees(r_lon_local) + InitialData['longitude']
    return (lat, lon)

def heading_to_angle(_h):
    """ Get Earth coordinate angle from heading """
    _a = (360 + 90 - _h) % 360
    return _a

def angle_to_heading(_a):
    """ Get heading from Earth coordinate angle """
    _h = (360 + 90 - _a) % 360
    return _h

def get_xy_from_xa_ya(_xa, _ya):
    """ get Earth X and Y from xa, ya measured from start point
        in the runway coordiante system """
    # angle to runway in Earth coordinate system in degrees
    runway_angle = heading_to_angle(InitialData['heading'])
    # angle to runway in Earth coordinate system in radians
    r_runway_angle = math.radians(runway_angle)
    # distance from starting point
    dist_a = get_distance(0, 0, _xa, _ya)
    # angle to the point in runway coordinate system in radians
    r_angle_a = math.atan2(_ya, _xa)
    # angle between runway and the point
    r_runway_angle_a = r_angle_a - math.pi/2
    # angle to the point in Earth coordiante system
    r_angle = r_runway_angle + r_runway_angle_a

    _x = dist_a * math.cos(r_angle)
    _y = dist_a * math.sin(r_angle)
    return(_x, _y)

def get_distance(_x0, _y0, _x1, _y1):
    """ Calculate distance between 2 points """
    delta_x = _x1 - _x0
    delta_y = _y1 - _y0
    distance = math.sqrt(delta_x*delta_x + delta_y*delta_y)
    return distance

def get_heading(_x0, _y0, _x1, _y1):
    """ Calculate azimuth from x0, y0 to x1, y1 """
    delta_x = _x1 - _x0
    delta_y = _y1 - _y0
    r_angle = math.atan2(delta_y, delta_x)

    angle = math.degrees(r_angle)

    heading = angle_to_heading(angle)
    return heading

def get_heading_diff(_h1, _h2):
    """ Calculate heading difference from _h1 to _h2
        set _h1 to current, _h2 to required """
    _h1 = _h1 % 360
    _h2 = _h2 % 360
    hdiff = _h2 - _h1 + (_h2 < _h1) * 360
    if hdiff > 180:
        hdiff -= 360
    return hdiff

def get_heading_diff2(_h1, _h2, _dir):
    """ Calculate heading difference with respect to turn direction """
    hdiff = get_heading_diff(_h1, _h2)
    if _dir == 'left' and hdiff > 0:
        hdiff -= 360
    if _dir == 'right' and hdiff < 0:
        hdiff += 360
    return hdiff

def get_runway_center_dist(_lat, _lon, runway_heading):
    """ Calculate distance from runway center to the plane.
        If plane is deviated counter-clock-wise, dist is positive.
        If plane is deviated clock-wise, dist is negative. """
    # Current plane point
    (_x, _y) = get_xy_from_lat_lon(_lat, _lon)

    r_angle_to_plane = math.atan2(_y, _x)
    angle_to_plane = math.degrees(r_angle_to_plane)
    heading_to_plane = angle_to_heading(angle_to_plane)
    # angle from runway center to direction to plane
    _a_delta = get_heading_diff(heading_to_plane, runway_heading)

    distance_from_start = get_distance(0, 0, _x, _y)
    center_dev = distance_from_start * math.sin(math.radians(_a_delta))
    if VERBOSE:
        print("center dev: {}".format(center_dev))
    return center_dev

def get_runway_center_correction(speed, center_dist):
    """ Get correction angle on runway.
        If we need to go clock-wise, correction is negative.
        If we need to go counter-clock-wise, correction is positive. """
    # Knots to m/s
    speed_meter_ps = speed * 0.514444
    # get the angle to center on railway in 20 sec
    center_correction = math.degrees(math.atan(center_dist/(speed_meter_ps*20 + 1)))
    return center_correction

def get_runway_center_heading(_lat, _lon, _speed):
    """ Get heading to runway center """
    center_dist = get_runway_center_dist(_lat, _lon, InitialData['heading'])
    center_correction = get_runway_center_correction(_speed, center_dist)
    center_heading = InitialData['heading'] + center_correction
    return center_heading

def get_climb_for_glissade(_xa, _ya, _lat, _lon, speed, _alt):
    """ Calculate climbing required to get to XA, YA
        from current lat and lon at current speed and altitude """
    # Calculate required vertical speed (feet per sec)

    # 1 knot is 1,68781 feet per sec
    speed_fps = speed * 1.68781
    # get distance-to-the-start-point/alt ratio. 1 meter is 3,28084 feets
    (_x, _y) = get_xy_from_lat_lon(_lat, _lon)
    (land_x, land_y) = get_xy_from_xa_ya(_xa, _ya)
    distance_to_landpoint = get_distance(_x, _y, land_x, land_y)
    dist_alt_ratio = distance_to_landpoint * 3.28084 / _alt
    climb_required = (-1) * speed_fps / dist_alt_ratio

    if VERBOSE:
        print("to land: {:+08.2f}, heading: {:+05.2f}"
          .format(distance_to_landpoint, SetPoints['heading']))

    return climb_required

def get_climb_by_altitude():
    """ Set climb based on altitude """
    altitude_dev = SetPoints['altitude'] - CurrentData['altitude']
    climb = s_shape(altitude_dev, 10, limit = 10)
    return climb

def initial_state():
    """ Process initial state """
    InitialData['heading'] = CurrentData['heading']
    InitialData['altitude'] = CurrentData['altitude']
    InitialData['elevation'] = CurrentData['elevation']
    InitialData['ground_alt'] = CurrentData['ground_alt']
    InitialData['latitude'] = CurrentData['latitude']
    InitialData['longitude'] = CurrentData['longitude']

    SetPoints['altitude'] = None
    SetPoints['bank'] = None
    SetPoints['heading'] = None
    SetPoints['speed'] = None
    SetPoints['climb'] = None
    SetPoints['flaps'] = 0.0
    SetPoints['pitch'] = None


    if CurrentData['rpm'] > Settings['engine_on_rpm']:
        return True
    return False

def setalt_state():
    """ Process set alt state """
    SetPoints['altitude'] = get_cur_arg()
    return True

def setspeed_state():
    """ Process set speed state """
    SetPoints['speed'] = get_cur_arg()
    return True

def takeoff_state():
    """ Process takeoff state """
    SetPoints['heading'] = get_runway_center_heading(CurrentData['latitude'],
                                                     CurrentData['longitude'],
                                                     CurrentData['speed'])
    if CurrentData['speed'] > Settings['takeoffspeed']:
        return True
    return False

def climbing_state():
    """ Process climbing state """
    SetPoints['climb'] = get_climb_by_altitude()
    if CurrentData['altitude'] > SetPoints['altitude']:
        return True
    return False

def sethead_state():
    """" Process set heading state """
    SetPoints['climb'] = get_climb_by_altitude()
    SetPoints['heading'] = get_cur_arg()[0]
    heading_diff = get_heading_diff(CurrentData['heading'], SetPoints['heading'])
    if abs(heading_diff) < Settings['turn_headingdelta']:
        return True
    return False

def level_state():
    """" Process level state """
    SetPoints['climb'] = get_climb_by_altitude()
    (_x, _y) = get_xy_from_lat_lon(CurrentData['latitude'],
                                       CurrentData['longitude'])
    (_xa, _ya) = get_cur_arg()
    (_x1, _y1) = get_xy_from_xa_ya(_xa, _ya)
    heading_to_point = get_heading(_x, _y, _x1, _y1)
    SetPoints['heading'] = heading_to_point

    distance = get_distance(_x, _y, _x1, _y1)
    heading_error = get_heading_diff(CurrentData['heading'], heading_to_point)
    if VERBOSE:
        print("Distance: {:5.2f}, heading_error: {:+05.2f}".format(distance, heading_error))
    if distance < Settings['level_distancedelta']:
        return True
    return False

def descending_state():
    """ Process descending state """
    # Calculate required vertical speed (feet per sec)

    # 1 knot is 1,68781 feet per sec
    # get distance-to-the-start-point/alt ratio. 1 meter is 3,28084 feets
    (land_xa, land_ya) = get_cur_arg()
    SetPoints['climb'] = get_climb_for_glissade(land_xa, land_ya,
                                               CurrentData['latitude'],
                                               CurrentData['longitude'],
                                               CurrentData['speed'],
                                               CurrentData['altitude'])

    SetPoints['heading'] = get_runway_center_heading(CurrentData['latitude'],
                                                     CurrentData['longitude'],
                                                     CurrentData['speed'])

    SetPoints['altitude'] = None
    SetPoints['flaps'] = 0.75
    SetPoints['speed'] = Settings['glissadespeed']
    if CurrentData['ground_alt'] < Settings['landingalt']:
        return True
    return False

def landing_state():
    """ Process landing state 2
        Make flight horisontal at landingalt and drop speed """

    SetPoints['heading'] = get_runway_center_heading(CurrentData['latitude'],
                                                     CurrentData['longitude'],
                                                     CurrentData['speed'])


    if SetPoints['climb'] is not None:
        SetPoints['climb'] = 0.0
        SetPoints['speed'] = Settings['landing_speed']

    if CurrentData['pitch'] > Settings['landingpitch']:
        SetPoints['climb'] = None
        SetPoints['flaps'] = 0.50


    if VERBOSE:
        print("ground_alt: {}".format(CurrentData['ground_alt']))
    if CurrentData['ground_alt'] < Settings['dropspeed_ground_alt']:
        SetPoints['speed'] = 0.0
        return True
    return False

def process_data(inputs):
    # pylint: disable=too-many-statements
    # pylint: disable=too-many-branches
    """ Main processing function """
    CurrentData['heading'] = float(inputs['Heading'])
    CurrentData['speed'] = float(inputs['Speed'])
    CurrentData['altitude'] = float(inputs['Altitude'])
    CurrentData['bank'] = float(inputs['Bank'])
    CurrentData['rpm'] = int(inputs['RPM'])
    CurrentData['latitude'] = float(inputs['Latitude'])
    CurrentData['longitude'] = float(inputs['Longitude'])
    CurrentData['climb'] = float(inputs['Climb'])
    CurrentData['pitch'] = float(inputs['Pitch'])
    CurrentData['elevation'] = float(inputs['Elevation'])

    CurrentData['ground_alt'] = CurrentData['altitude'] - CurrentData['elevation']

    if not get_cur_flag():
        #for pid in PIDS:
        #    PIDS[pid].reset()
        set_cur_flag(True)
        if VERBOSE and get_cur_state() == 'initial':
            print(pprint.pprint(States))
        print("--------------------------------------------------------------")

    print("State: {}".format(get_cur_state()))

    go_next = False
    if get_cur_state()  == 'initial':
        go_next = initial_state()
    elif get_cur_state() == 'setalt':
        go_next = setalt_state()
    elif get_cur_state() == 'setspeed':
        go_next = setspeed_state()
    elif get_cur_state() == 'takeoff':
        go_next = takeoff_state()
    elif get_cur_state() == 'climbing':
        go_next = climbing_state()
    elif get_cur_state() == 'sethead':
        go_next = sethead_state()
    elif get_cur_state() == 'level':
        go_next = level_state()
    elif get_cur_state() == 'descending':
        go_next = descending_state()
    elif get_cur_state() == 'landing':
        go_next = landing_state()
    elif get_cur_state() == 'stop':
        print("Brakes!")
    else:
        error_pause("Unknown state", 100)

    if go_next:
        next_state()

    # Calculate deviations
    altitude_dev = 0.0
    climb_dev = 0.0
    pitch_dev = 0.0
    bank_dev = 0.0
    speed_dev = 0.0
    heading_dev = 0.0

    if SetPoints['heading'] is not None:
        if get_cur_state() == 'sethead':
            (_, direction) = get_cur_arg()
            heading_dev = get_heading_diff2(CurrentData['heading'], SetPoints['heading'], direction)
        else:
            heading_dev = get_heading_diff(CurrentData['heading'], SetPoints['heading'])

    if SetPoints['altitude'] is not None:
        altitude_dev = CurrentData['altitude'] - SetPoints['altitude']
    if SetPoints['climb'] is not None:
        climb_dev = CurrentData['climb'] - SetPoints['climb']
    if SetPoints['pitch'] is not None:
        pitch_dev = CurrentData['pitch'] - SetPoints['pitch']
    if SetPoints['bank'] is not None:
        bank_dev = CurrentData['bank'] - SetPoints['bank']
    if SetPoints['speed'] is not None:
        speed_dev = CurrentData['speed'] - SetPoints['speed']

    if VERBOSE:
        print("SetPoints: ", SetPoints)

    # Process deviations
    out = {}

    out['aileron'] = process_bank(bank_dev)
    out['rudder'] = process_heading(heading_dev)
    out['elevator'] = process_climb(climb_dev)
    out['throttle'] = process_speed(speed_dev)

    out['flaps'] = SetPoints['flaps']

    if VERBOSE:
        print("Deviations: Heading {:+06.2f}, altitude {:+06.2f}, "
          "bank {:+06.2f}, speed {:+06.2f}, climb {:+06.2f}, pitch: {:+06.2f}"
          .format(heading_dev, altitude_dev, bank_dev, speed_dev, climb_dev, pitch_dev))

    return out

if __name__ == "__main__":
    InitialData['heading'] = 232.8
    InitialData['latitude'] = 21.329819
    InitialData['longitude'] = -157.907042
    _LAT = 021.34550579649
    _LON = -157.88530113263

    for _ya in range(500, 5000, 500):
        (_x, _y) = get_xy_from_xa_ya(0,-_ya)
        _h = get_heading(_x, _y, 0, 0)
        (_lat, _lon) = get_lat_lon_from_xy(_x, _y)
        print(_ya, _x, _y, "({}, {})".format(_lat, _lon), _h)
