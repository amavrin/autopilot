""" Process data locally """

import time
import math
import pprint
import configparser
import re
from simple_pid import PID
# from numpy import clip

Settings = {}
SetPoints = {}
CurrentData = {}
PIDS = {}
Runway = {}
Flight = {}

Out = {}
Out['aileron'] = 0.0
Out['elevator'] = 0.0
Out['rudder'] = 0.0
Out['throttle'] = 0.0
Out['flaps'] = 0.0
Out['brake'] = True

Deviations = {}
Deviations['altitude'] = 0.0
Deviations['climb'] = 0.0
Deviations['pitch'] = 0.0
Deviations['speed'] = 0.0
Deviations['heading'] = 0.0

# initial, takeoff, climbing, sethead,
# setspeed
# level, descending, landing, stop
States = {}

VERBOSE = True

def error_pause(_s,_t):
    """ Print message and sleep """
    print("ERROR: {}".format(_s))
    time.sleep(_t)

def get_rw_start_lat_lon(runway = 'base'):
    """ get starting lat, lon for the runway """
    (_lat, _lon) = (float(Runway[runway][0]), float(Runway[runway][1]))
    return (_lat, _lon)

def get_rw_end_lat_lon(runway = 'base'):
    """ get ending lat, lon for the runway """
    (_lat, _lon) = (float(Runway[runway][2]), float(Runway[runway][3]))
    return (_lat, _lon)

def get_rw_elev(runway = 'base'):
    """ get starting elevation for the runway """
    elev = (float(Runway[runway][4]))
    return elev

def get_rw_head(runway = 'base'):
    """ get the heading for the runway """
    (_lat0, _lon0) = get_rw_start_lat_lon(runway)
    (_lat1, _lon1) = get_rw_end_lat_lon(runway)
    (_x0, _y0) = get_xy_from_lat_lon(_lat0, _lon0)
    (_x1, _y1) = get_xy_from_lat_lon(_lat1, _lon1)
    head = get_heading(_x0, _y0, _x1, _y1)
    return head

def parse_config():
    """ read config file """
    config = configparser.ConfigParser()
    config.read('process_local.conf')
    Settings['takeoffspeed'] = config.getfloat('settings', 'takeoffspeed')
    Settings['prelanding_speed'] = config.getfloat('settings', 'prelanding_speed')
    Settings['targetspeed'] = config.getfloat('settings', 'targetspeed')
    Settings['targetalt'] = config.getfloat('settings', 'targetalt')
    Settings['landingalt'] = config.getfloat('settings', 'landingalt')
    Settings['landing_speed'] = config.getfloat('settings', 'landing_speed')
    Settings['landingpitch'] = config.getfloat('settings', 'landingpitch')
    Settings['landingclimb'] = config.getfloat('settings', 'landingclimb')
    Settings['dropspeed_ground_alt'] = config.getfloat('settings', 'dropspeed_ground_alt')
    Settings['glissadealt'] = config.getfloat('settings', 'glissadealt')
    Settings['glissadespeed'] = config.getfloat('settings', 'glissadespeed')
    Settings['engine_on_rpm'] = config.getint('settings', 'engine_on_rpm')
    Settings['turnbank'] = config.getfloat('settings', 'turnbank')
    Settings['turn_headingdelta'] = config.getfloat('settings', 'turnbank')
    Settings['level_distancedelta'] = config.getfloat('settings', 'level_distancedelta')
    Settings['takeoff_runway'] = config['settings']['takeoff_runway']
    Settings['landing_runway'] = config['settings']['landing_runway']

    Flight['steps'] = re.split(', *|,', config['flight']['program'])

    for runway in config['runways']:
        Runway[runway] = re.split(', *|,', config['runways'][runway])

    Runway['base'] = Runway[Settings['takeoff_runway']]

def construct_program():
    """ Construct flight program """

    States['program'] = []
    States['program'].append({ 'name': 'initial' })
    States['program'].append({ 'name': 'setspeed', 'arg': Settings['targetspeed'] })
    States['program'].append({ 'name': 'setalt', 'arg': Settings['targetalt'] })
    States['program'].append({ 'name': 'takeoff' })
    States['program'].append({ 'name': 'climbing' })

    for step in Flight['steps']:
        if step == 'n_setalt':
            for _n in range(0, 20, 2):
                States['program'].append({ 'name': 'level',
                        'arg': (0, 3000 + _n * 1000, 'runway') })
                States['program'].append({ 'name': 'setalt', 'arg': 300 })
                States['program'].append({ 'name': 'level',
                        'arg': (0, 3000 + (_n+1) * 1000, 'runway') })
                States['program'].append({ 'name': 'setalt', 'arg': 150 })
        elif step == 'zig_zag':
            for _n in range(1,6,2):
                States['program'].append({ 'name': 'level',
                        'arg': (100, 4000 + _n * 1500, 'runway') })
                States['program'].append({ 'name': 'level',
                        'arg': (-100, 4000 + (_n+1) * 1500, 'runway') })
                States['program'].append({ 'name': 'setspeed', 'arg': 80 + _n * 5 })
        elif step == 'turns':
            States['program'].append({ 'name': 'sethead', 'arg': (200, 'left') })
            States['program'].append({ 'name': 'sethead', 'arg': (340, 'right') })
            States['program'].append({ 'name': 'sethead', 'arg': (200, 'left') })
            States['program'].append({ 'name': 'sethead', 'arg': (340, 'right') })
            States['program'].append({ 'name': 'sethead', 'arg': (200, 'left') })
            States['program'].append({ 'name': 'sethead', 'arg': (340, 'right') })
        elif step == 'round':
            rw_head = get_rw_head()
            States['program'].append({ 'name': 'sethead', 'arg': ((rw_head + 180)%360, 'left') })
            States['program'].append({ 'name': 'level', 'arg': (-1100, -5000, 'runway') })
        elif step == 'to_runway':
            States['program'].append({ 'name': 'set_runway', 'arg': Settings['landing_runway'] })
        else:
            error_pause("Choose correct program", 1000)

    # Finish
    States['program'].append({ 'name': 'level_head', 'arg': (0, -5000) })
    # Turn to the glissade, take off speed
    States['program'].append({ 'name': 'set_runway', 'arg': Settings['landing_runway'] })
    States['program'].append({ 'name': 'setspeed', 'arg': Settings['prelanding_speed'] })
    States['program'].append({ 'name': 'setalt', 'arg': 600 })
    States['program'].append({ 'name': 'level', 'arg': (0, -4000, 'runway') })
    States['program'].append({ 'name': 'level', 'arg': (0, -2500, 'runway') })
    # start glissade
    States['program'].append({ 'name': 'descending', 'arg': (0, 0) })
    States['program'].append({ 'name': 'landing' })
    States['program'].append({ 'name': 'stop' })
    ################


def init():
    # pylint: disable=too-many-statements
    # pylint: disable=too-many-branches
    """ Init data for local processing """

    parse_config()
    construct_program()

    PIDS['rudder'] = PID(0.0, 0.0, 0.0, setpoint=0)
    PIDS['aileron'] = PID(0.0, 0.0, 0.0, setpoint=0)
    PIDS['throttle'] = PID(0.0, 0.0, 0.0, setpoint=0)
    PIDS['elevator'] = PID(0.0, 0.0, 0.0, setpoint=0)
    States['current'] = 0

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

def get_rudder(heading_dev):
    """ Heading processing. Also set required bank angle """
    k_prop = 0.0
    k_int = 0.0
    k_der = 0.0
    low = 0.0
    high = 0.0

    if get_cur_state() in ('takeoff', 'stop', 'landing'):
        (k_prop, k_int, k_der) = (0.005, 0.02, 0.02)
        (low, high) = (-0.3, 0.3)
    elif get_cur_state() in ('level', 'climbing', 'sethead', 'descending'):
        # 1 at heading_dev == 0, near 0 at large heading_dev
        k_prop = bellshape(heading_dev, 10, limit = 0.03, zero = False)
        #k_int = bellshape(heading_dev, 10, limit = 0.001, zero = False)
        k_int = 0
        k_der = 0.0
        (low, high) = (-0.3, 0.3)

    PIDS['rudder'].tunings = (k_prop, k_int, k_der)
    PIDS['rudder'].output_limits = (low, high)

    rudder = PIDS['rudder']((-1)*heading_dev)
    return rudder

def get_aileron(heading_dev):
    """ Sample bank processing """

    SetPoints['bank'] = 0.0
    if get_cur_state() in ('level', 'climbing', 'sethead', 'descending'):
        # 0 at heading_dev == 0, near 1 at large heading_dev, with the same sign as heading_dev
        bank_prop = s_shape(heading_dev, 8)
        SetPoints['bank'] = bank_prop*Settings['turnbank']

    Deviations['bank'] = CurrentData['bank'] - SetPoints['bank']

    k_prop = 0.0
    k_int = 0.0
    k_der = 0.0
    low = 0.0
    high = 0.0

    k_prop = 0.04
    k_int = prop(50, 0.003, 100, 0.012, CurrentData['speed'], y_min = 0.0)
    k_der = prop(100, 0.01, 50, 0.05, CurrentData['speed'])
    (low, high) = (-0.4, 0.4)

    PIDS['aileron'].tunings = (k_prop, k_int, k_der)
    PIDS['aileron'].output_limits = (low, high)

    aileron = PIDS['aileron'](Deviations['bank'])
    return aileron

def get_elevator(climb_dev):
    """ Altitude processing on glissade """
    k_prop = 0.0
    k_int = 0.0
    k_der = 0.0
    low = 0.0
    high = 0.0

    if get_cur_state() == 'landing':
        k_prop = 0.01
        k_int = prop(70, 0.01, 40, 0.015, CurrentData['speed'], y_min = 0.0)
        k_der = 0.002
        (low, high) = (-0.3, 0.1)
    else:
        k_prop = prop(60, 0.008, 100, 0.004, CurrentData['speed'], y_min = 0.001)
        k_int = prop(60, 0.003, 100, 0.001, CurrentData['speed'], y_min = 0.0)
        k_der = prop(60, 0.008, 100, 0.004, CurrentData['speed'], y_min = 0.0)
        (low, high) = (-0.2, 0.1)

    PIDS['elevator'].tunings = (k_prop, k_int, k_der)
    PIDS['elevator'].output_limits = (low, high)

    elevator = PIDS['elevator'](- climb_dev)
    return elevator

def get_throttle(speed_dev):
    """ Flight speed processing """
    PIDS['throttle'].tunings = (0.01, 0.003, 0.01)
    PIDS['throttle'].output_limits = (0.002, 1.0)
    return PIDS['throttle'](speed_dev)

def get_xy_from_lat_lon(_lat, _lon):
    """ Calculate relative X and Y based on latitude and longitude """
    (base_lat, base_lon) = get_rw_start_lat_lon('base')
    r_lat_local = math.radians(_lat - base_lat)
    r_lon_local = math.radians(_lon - base_lon)
    _x = 6371000 * r_lon_local * math.cos(math.radians(base_lat))
    _y = 6371000 * r_lat_local
    return (_x, _y)

def get_lat_lon_from_xy(_x, _y):
    """ Calculate lat and lon from X and Y """
    (base_lat, base_lon) = get_rw_start_lat_lon('base')
    r_lat_local = _y / 6371000
    r_lon_local = _x / 6371000 / math.cos(r_lat_local + math.radians(base_lat))
    lat = math.degrees(r_lat_local) + base_lat
    lon = math.degrees(r_lon_local) + base_lon
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
    base_head = get_rw_head('base')
    runway_angle = heading_to_angle(base_head)
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
    base_head = get_rw_head('base')
    center_dist = get_runway_center_dist(_lat, _lon, base_head)
    center_correction = get_runway_center_correction(_speed, center_dist)
    center_heading = base_head + center_correction
    return center_heading

def get_climb_for_glissade(_xa, _ya, _lat, _lon, speed, _alt):
    """ Calculate climbing required to get to XA, YA
        from current lat and lon at current speed and altitude """
    # Calculate required vertical speed (feet per sec)

    # 1 knot is 1,68781 feet per sec
    speed_fps = speed * 1.68781
    runway_elevation = get_rw_elev()
    # get distance-to-the-start-point/alt ratio. 1 meter is 3,28084 feets
    (_x, _y) = get_xy_from_lat_lon(_lat, _lon)
    (land_x, land_y) = get_xy_from_xa_ya(_xa, _ya)
    distance_to_landpoint = get_distance(_x, _y, land_x, land_y)
    dist_alt_ratio = distance_to_landpoint * 3.28084 / (_alt - runway_elevation)
    climb_required = (-1) * speed_fps / dist_alt_ratio

    if VERBOSE:
        print("to land: {:+08.2f}, heading: {:+05.2f}"
          .format(distance_to_landpoint, SetPoints['heading']))

    return climb_required

def get_climb_by_altitude():
    """ Set climb based on altitude """
    altitude_dev = SetPoints['altitude'] - CurrentData['altitude']
    climb_limit = prop(0, 0, 100, 10, CurrentData['speed'])
    climb = s_shape(altitude_dev, 10, limit = climb_limit)
    return climb

def initial_state():
    """ Process initial state """
    SetPoints['altitude'] = None
    SetPoints['heading'] = None
    SetPoints['speed'] = None
    SetPoints['climb'] = None
    SetPoints['flaps'] = 0.0
    SetPoints['pitch'] = None

    takeoff_rw = Settings['takeoff_runway']
    Runway[takeoff_rw][0] = CurrentData['latitude']
    Runway[takeoff_rw][1] = CurrentData['longitude']

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

def set_runway_state():
    """ set base runway """
    runway = get_cur_arg()
    Runway['base'] = Runway[runway]
    return True

def takeoff_state():
    """ Process takeoff state """
    SetPoints['heading'] = get_runway_center_heading(CurrentData['latitude'],
                                                     CurrentData['longitude'],
                                                     CurrentData['speed'])
    calc_devs()

    # Process deviations
    Out['rudder'] = get_rudder(Deviations['heading'])
    Out['throttle'] = 1.0
    Out['brake'] = False

    if CurrentData['speed'] > Settings['takeoffspeed']:
        return True
    return False

def climbing_state():
    """ Process climbing state """
    SetPoints['climb'] = get_climb_by_altitude()

    calc_devs()

    # Process deviations
    Out['rudder'] = get_rudder(Deviations['heading'])
    Out['aileron'] = get_aileron(Deviations['heading'])
    Out['elevator'] = get_elevator(Deviations['climb'])
    Out['throttle'] = get_throttle(Deviations['speed'])

    if CurrentData['altitude'] > SetPoints['altitude']:
        return True
    return False

def sethead_state():
    """" Process set heading state """
    SetPoints['climb'] = get_climb_by_altitude()
    SetPoints['heading'] = get_cur_arg()[0]
    heading_diff = get_heading_diff(CurrentData['heading'], SetPoints['heading'])

    calc_devs()

    # Process deviations
    Out['rudder'] = get_rudder(Deviations['heading'])
    Out['aileron'] = get_aileron(Deviations['heading'])
    Out['elevator'] = get_elevator(Deviations['climb'])
    Out['throttle'] = get_throttle(Deviations['speed'])

    if abs(heading_diff) < Settings['turn_headingdelta']:
        return True
    return False

def _get_circle_params(_x_cur, _y_cur, current_angle, radius, side):
    """ Get circle center coordinates
        given the current coordinates and the angle """
    if side == 'left':
        angle_diff = math.pi/2
    elif side == 'right':
        angle_diff = - math.pi/2
    else:
        error_pause("wrong parameter side: {}".format(side), 100)

    center_angle = math.radians(current_angle) + angle_diff
    center_x = _x_cur + math.cos(center_angle)*radius
    center_y = _y_cur + math.sin(center_angle)*radius
    return (center_x, center_y)

def _get_circle_tangent_angle(_c0x, _c0y, c0_dir, _c1x, _c1y, c1_dir, radius):
    """ Get the tangent angle to 2 circles with respect to circle direction """
    # angles from left and right current circles to preliminary dest circles
    tangent_angle = None
    center_angle = math.atan2(_c1y - _c0y, _c1x - _c0x)
    if c0_dir == c1_dir:
        tangent_angle = center_angle
    else:
        center_dist = get_distance(_c0x, _c0y, _c1x, _c1y)
        if center_dist >= 2*radius:
            alpha = math.asin(radius/(center_dist/2))
            if c0_dir == 'left':
                tangent_angle = center_angle + alpha
            else:
                tangent_angle = center_angle - alpha
    ###print(c0_dir, c1_dir, math.degrees(tangent_angle))
    return tangent_angle

def get_best_tangent_heading(_x0, _y0, head0, _x1, _y1, head1, radius):
    # pylint: disable=too-many-locals
    """ Calculate the best tangent angle of turn """
    # Calculate left and right circles from where we are
    _c0 = {}
    _c1 = {}
    angle0 = heading_to_angle(head0)
    angle1 = heading_to_angle(head1)

    for _dir in ('left', 'right'):
        _c0[_dir] = _get_circle_params(_x0, _y0, angle0, radius, _dir)
        _c1[_dir] = _get_circle_params(_x1, _y1, angle1, radius, _dir)

    best_dir0 = None
    best_dir1 = None
    best_tangent_head = None
    x_on_point = None
    y_on_point = None
    min_head_diff_summ = 2*360

    for c0_dir in ('left', 'right'):
        for c1_dir in ('left', 'right'):
            tangent_angle = _get_circle_tangent_angle(
                                        _c0[c0_dir][0],
                                        _c0[c0_dir][1],
                                        c0_dir,
                                        _c1[c1_dir][0],
                                        _c1[c1_dir][1],
                                        c1_dir,
                                        radius
                                    )
            if tangent_angle is None:
                continue
            tangent_head = angle_to_heading(math.degrees(tangent_angle))
            turn0 = get_heading_diff2(head0, tangent_head, c0_dir)
            turn1 = get_heading_diff2(tangent_head, head1, c1_dir)
            head_diff_summ = abs(turn0) + abs(turn1)
            if head_diff_summ < min_head_diff_summ:
                min_head_diff_summ = head_diff_summ
                best_dir0 = c0_dir
                best_dir1 = c1_dir
                best_tangent_head = tangent_head
                # the tangent point on target circle
                if best_dir1 == 'right':
                    sign = 1
                else:
                    sign = -1
                x_on_point = _c1[c1_dir][0] + math.cos(sign * math.pi/2 + tangent_angle)*radius
                y_on_point = _c1[c1_dir][1] + math.sin(sign * math.pi/2 + tangent_angle)*radius

    return(best_dir0, best_dir1, best_tangent_head, x_on_point, y_on_point)


def level_head_state():
    # pylint: disable=too-many-locals
    """ Go to the point and set head """
    current_heading = CurrentData['heading']
    (_x1a, _y1a) = get_cur_arg()
    (_x1, _y1) = get_xy_from_xa_ya(_x1a, _y1a)
    target_head = get_rw_head()
    target_angle = heading_to_angle(target_head)
    _x0, _y0 = get_xy_from_lat_lon(CurrentData['latitude'],
                                 CurrentData['longitude'])

    # Turn radius
    circle_radius = 800
    # Preliminary distance
    pre_distance = 1000

    # Calculate preliminary dest point
    _x1pre = _x1 - math.cos(math.radians(target_angle))*pre_distance
    _y1pre = _y1 - math.sin(math.radians(target_angle))*pre_distance

    (dir0, dir1, turn_head, x_on_point, y_on_point) = get_best_tangent_heading(_x0, _y0,
                                                current_heading,
                                                _x1pre, _y1pre, target_head,
                                                circle_radius)

    pos = States['current']
    States['program'].insert(pos + 1, {'name': 'sethead', 'arg': (turn_head, dir0)})
    States['program'].insert(pos + 2, {'name': 'level', 'arg': (x_on_point, y_on_point, 'earth')})
    States['program'].insert(pos + 3, {'name': 'sethead', 'arg': (target_head, dir1)})
    States['program'].insert(pos + 4, {'name': 'level', 'arg': (_x1a, _y1a, 'runway')})

    return True

def level_state():
    """" Process level state """
    SetPoints['climb'] = get_climb_by_altitude()
    (_x, _y) = get_xy_from_lat_lon(CurrentData['latitude'],
                                       CurrentData['longitude'])
    (_xarg, _yarg) = get_cur_arg()[0:2]
    coord = get_cur_arg()[2]

    if coord == 'runway':
        (_x1, _y1) = get_xy_from_xa_ya(_xarg, _yarg)
    else:
        (_x1, _y1) = (_xarg, _yarg)

    heading_to_point = get_heading(_x, _y, _x1, _y1)
    SetPoints['heading'] = heading_to_point

    distance = get_distance(_x, _y, _x1, _y1)
    heading_error = get_heading_diff(CurrentData['heading'], heading_to_point)
    if VERBOSE:
        print("Distance: {:5.2f}, heading_error: {:+05.2f}".format(distance, heading_error))

    calc_devs()

    # Process deviations
    Out['rudder'] = get_rudder(Deviations['heading'])
    Out['aileron'] = get_aileron(Deviations['heading'])
    Out['elevator'] = get_elevator(Deviations['climb'])
    Out['throttle'] = get_throttle(Deviations['speed'])

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

    calc_devs()

    # Process deviations
    Out['rudder'] = get_rudder(Deviations['heading'])
    Out['aileron'] = get_aileron(Deviations['heading'])
    Out['elevator'] = get_elevator(Deviations['climb'])
    Out['throttle'] = get_throttle(Deviations['speed'])
    Out['flaps'] = SetPoints['flaps']

    if CurrentData['ground_alt'] < Settings['landingalt']:
        return True
    return False

def landing_state():
    """ Process landing state in 2 phases:
        1. Make flight horisontal at landingalt, drop speed and wait for pith
        2. Lower slowly with constant pitch """

    Out['brake'] = True

    SetPoints['heading'] = get_runway_center_heading(CurrentData['latitude'],
                                                     CurrentData['longitude'],
                                                     CurrentData['speed'])



    if 'landing_phase' not in CurrentData:
        CurrentData['landing_phase'] = 1

    if CurrentData['landing_phase'] == 1:
        SetPoints['climb'] = -0.5
        SetPoints['speed'] = Settings['landing_speed']
        SetPoints['flaps'] = 1.0
        elev_dev_kind = 'climb'
        throt_dev_kind = 'speed'
        if CurrentData['pitch'] > Settings['landingpitch']:
            CurrentData['landing_phase'] = 2

    if CurrentData['landing_phase'] == 2:
        SetPoints['climb'] = Settings['landingclimb']
        SetPoints['speed'] = 0.0
        SetPoints['pitch'] = Settings['landingpitch']
        elev_dev_kind = 'pitch'
        throt_dev_kind = 'climb'

    calc_devs()
    Out['elevator'] = get_elevator(Deviations[elev_dev_kind])
    Out['throttle'] = get_throttle(Deviations[throt_dev_kind])

    if VERBOSE:
        print("elev kind: {}, thot kind: {}, ground_alt: {}"
                .format(elev_dev_kind, throt_dev_kind, CurrentData['ground_alt']))

    # Process deviations
    Out['rudder'] = get_rudder(Deviations['heading'])
    Out['aileron'] = get_aileron(Deviations['heading'])
    Out['flaps'] = SetPoints['flaps']

    if CurrentData['ground_alt'] < Settings['dropspeed_ground_alt']:
        Out['throttle'] = 0.0
        return True
    return False

def stop_state():
    """ On the runway """
    SetPoints['heading'] = get_runway_center_heading(CurrentData['latitude'],
                                                     CurrentData['longitude'],
                                                     CurrentData['speed'])
    calc_devs()

    Out['aileron'] = 0.0
    Out['elevator'] = 0.0
    Out['rudder'] = get_rudder(Deviations['heading'])
    Out['throttle'] = 0.0
    Out['flaps'] = 0.0
    Out['brake'] = True


def calc_devs():
    """ Calculate deviations """
    # Calculate deviations
    Deviations['altitude'] = 0.0
    Deviations['climb'] = 0.0
    Deviations['pitch'] = 0.0
    Deviations['speed'] = 0.0
    Deviations['heading'] = 0.0

    if SetPoints['heading'] is not None:
        if get_cur_state() == 'sethead':
            (_, direction) = get_cur_arg()
            Deviations['heading'] = get_heading_diff2(CurrentData['heading'],
                    SetPoints['heading'], direction)
        else:
            Deviations['heading'] = get_heading_diff(CurrentData['heading'],
                    SetPoints['heading'])

    if SetPoints['altitude'] is not None:
        Deviations['altitude'] = CurrentData['altitude'] - SetPoints['altitude']
    if SetPoints['climb'] is not None:
        Deviations['climb'] = CurrentData['climb'] - SetPoints['climb']
    if SetPoints['pitch'] is not None:
        Deviations['pitch'] = CurrentData['pitch'] - SetPoints['pitch']
    if SetPoints['speed'] is not None:
        Deviations['speed'] = CurrentData['speed'] - SetPoints['speed']

    if VERBOSE:
        print("Deviations: ", ["{}: {:+.3f}".format(x, Deviations[x]) for x in Deviations])

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
    elif get_cur_state() == 'set_runway':
        go_next = set_runway_state()
    elif get_cur_state() == 'takeoff':
        go_next = takeoff_state()
    elif get_cur_state() == 'climbing':
        go_next = climbing_state()
    elif get_cur_state() == 'sethead':
        go_next = sethead_state()
    elif get_cur_state() == 'level':
        go_next = level_state()
    elif get_cur_state() == 'level_head':
        go_next = level_head_state()
    elif get_cur_state() == 'descending':
        go_next = descending_state()
    elif get_cur_state() == 'landing':
        go_next = landing_state()
    elif get_cur_state() == 'stop':
        stop_state()
    else:
        error_pause("Unknown state", 100)

    if go_next:
        next_state()

    if VERBOSE:
        print("SetPoints: ", SetPoints)

    return Out

def test_get_best_tangent_heading():
    # pylint: disable=too-many-locals
    # pylint: disable=invalid-name
    """ main """
    parse_config()
    _x0 = 0
    _y0 = 0
    current_heading = 270
    _x1pre = 1000
    _y1pre = -1000
    target_head = 270
    circle_radius = 300

    (dir0, dir1, turn_head, x_on_point, y_on_point) = get_best_tangent_heading(_x0, _y0,
                                                current_heading,
                                                _x1pre, _y1pre, target_head,
                                                circle_radius)
    print(dir0, dir1, turn_head)
    _, ax = plt.subplots()
    ax.plot([_x0, _x1pre, x_on_point],
            [_y0, _y1pre, y_on_point],
            'ro')


    turn_angle = heading_to_angle(turn_head)
    print(turn_head, turn_angle)
    ax.axline((x_on_point, y_on_point), slope = math.tan(math.radians(turn_angle)),
              color='black')

    current_angle = heading_to_angle(current_heading)
    target_angle = heading_to_angle(target_head)

    ax.plot([_x0, _x0 + 300*math.cos(math.radians(current_angle))],
            [_y0, _y0 + 300*math.sin(math.radians(current_angle))]
            )

    ax.plot([_x1pre, _x1pre + 300*math.cos(math.radians(target_angle))],
            [_y1pre, _y1pre + 300*math.sin(math.radians(target_angle))]
            )

    rA0left = math.radians(current_angle) + math.pi/2
    rA0right = math.radians(current_angle) - math.pi/2
    Xr0left = _x0 + math.cos(rA0left)*circle_radius
    Yr0left = _y0 + math.sin(rA0left)*circle_radius
    Xr0right = _x0 + math.cos(rA0right)*circle_radius
    Yr0right = _y0 + math.sin(rA0right)*circle_radius

    rA1left = math.radians(target_angle) + math.pi/2
    rA1right = math.radians(target_angle) - math.pi/2
    Xr1left = _x1pre + math.cos(rA1left)*circle_radius
    Yr1left = _y1pre + math.sin(rA1left)*circle_radius
    Xr1right = _x1pre + math.cos(rA1right)*circle_radius
    Yr1right = _y1pre + math.sin(rA1right)*circle_radius

    c00 = plt.Circle((Xr0left, Yr0left), circle_radius)
    c01 = plt.Circle((Xr0right, Yr0right), circle_radius)
    c10 = plt.Circle((Xr1left, Yr1left), circle_radius, color='green')
    c11 = plt.Circle((Xr1right, Yr1right), circle_radius, color='green')

    ax.add_patch(c00)
    ax.add_patch(c01)
    ax.add_patch(c10)
    ax.add_patch(c11)

    plt.show()

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    test_get_best_tangent_heading()
