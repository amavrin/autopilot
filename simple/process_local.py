""" Process data locally """

import time
import math
from simple_pid import PID

Data = {}
SetPoints = {}
InitialData = {}
PIDS = {}
# initial, takeoff, climbing, turn180,
# level, turn180, descending, landing, stop
States = {}

def init():
    """ Init data for local processing """
    Data['takeoffspeed'] = 50.0
    Data['targetalt'] = 200.0
    Data['targetspeed'] = 100.0
    Data['engine_on_rpm'] = 100
    Data['turnbank'] = 15
    Data['turn_headingdelta'] = 3
    Data['level_distancedelta'] = 30
    Data['level_maxangle'] = 45
    PIDS['rudder'] = PID(0.01, 0.005, 0.001, setpoint=0)
    PIDS['aileron_level'] = PID(0.01, 0.03, 0.01, setpoint=0)
    PIDS['aileron_level'].output_limits = (-0.4, 0.4)
    PIDS['throttle'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['throttle'].output_limits = (0.3, 1.0)
    PIDS['elevator'] = PID(0.001, 0.01, 0.01, setpoint=0)
    PIDS['elevator'].output_limits = (-0.1, 0.01)
    #PIDS['elevator_turn'] = PID(0.001, 0.001, 0.02, setpoint=0)
    #PIDS['elevator_turn'].output_limits = (-0.2, 0.05)
    States['current'] = 0
    States['program'] = []
    States['program'].append({ 'name': 'initial' })
    States['program'].append({ 'name': 'takeoff' })
    States['program'].append({ 'name': 'climbing' })
    States['program'].append({ 'name': 'level', 'arg': (0, 4000) })
    #States['program'].append({ 'name': 'turn', 'arg': -180 })
    # 1000m right to start
    #States['program'].append({ 'name': 'level', 'arg': (1000, 0) })
    # 1000m right and 3000m back to start
    #States['program'].append({ 'name': 'level', 'arg': (1000, -3000) })
    States['program'].append({ 'name': 'turn', 'arg': -180 })
    States['program'].append({ 'name': 'descending' })
    States['program'].append({ 'name': 'landing' })
    States['program'].append({ 'name': 'stop' })

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

def process_heading(heading_dev):
    """ Sample heading processing """
    rudder = 0.0
    if get_cur_state() == 'takeoff' or \
       get_cur_state() == 'level' or \
       get_cur_state() == 'climbing':
        rudder = PIDS['rudder'](heading_dev)
    return rudder

def process_bank(bank_dev):
    """ Sample bank processing """
    if get_cur_state() == 'turn':
        if bank_dev >= 0:
            aileron = 0.0
        else:
            aileron = 0.015
    else:
        aileron = PIDS['aileron_level'](bank_dev)
    print("aileron: {}".format(aileron))
    return aileron

def process_altitude(altitude_dev):
    """ Sample altitude processing """
    elevator = 0.0
    if get_cur_state() != 'takeoff':
        elevator = PIDS['elevator'](altitude_dev)
    return elevator

def process_speed(speed_dev):
    """ Sample speed processing """
    return PIDS['throttle'](speed_dev)

def get_xy_from_lat_lon(_lat, _lon):
    """ Calculate X and Y based on latitude and longitude """
    r_lat = math.radians(_lat)
    r_lon = math.radians(_lon)
    _x = 6371000 * r_lon * math.cos(r_lat)
    _y = 6371000 * r_lat
    return (_x, _y)

def get_lat_lon_from_xy(_x, _y):
    """ Calculate lat and lon from X and Y """
    r_lat = _y / 6371000
    r_lon = _x / 6371000 / math.cos(r_lat)
    lat = math.degrees(r_lat)
    lon = math.degrees(r_lon)
    return (lat, lon)

def get_x_y_from_xa_ya(_xa, _ya):
    """ get Earth X and Y from xa, ya measured from start point
        in the runway coordiante system """
    # angle to runway in Earth coordinate system in degrees
    runway_angle = (360 + 90 - InitialData['heading']) % 360
    # angle to runway in Earth coordinate system in radians
    r_runway_angle = math.radians(runway_angle)
    # X and Y of starting point
    (_x0, _y0) = get_xy_from_lat_lon(InitialData['latitude'],
                                     InitialData['longitude'])
    # distance from starting point
    dist_a = math.sqrt(_xa*_xa + _ya*_ya)

    # angle to the point in runway coordinate system in radians
    # FIXME
    if _xa == 0 and _ya > 0:
        r_angle_a = math.pi/2
    if _xa == 0 and _ya < 0:
        r_angle_a = - math.pi/2
    if _xa > 0:
        r_angle_a = math.atan(_ya/_xa)
    if _xa < 0:
        r_angle_a = math.atan(_ya/_xa) + math.pi

    # angle to the point in Earth coordiante system
    r_angle = r_runway_angle - math.pi/2 + r_angle_a

    delta_x = dist_a * math.cos(r_angle)
    delta_y = dist_a * math.sin(r_angle)
    _x = _x0 + delta_x
    _y = _y0 + delta_y
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
    # FIXME
    if delta_x == 0:
        r_angle = math.pi/2
    elif delta_x > 0:
        r_angle = math.atan(delta_y/delta_x)
    else:
        r_angle = math.atan(delta_y/delta_x) + math.pi

    angle = math.degrees(r_angle)

    heading = (90 - angle + 360) % 360
    return heading

def get_runway_center_dist(_lat0, _lon0, _lat, _lon, alpha):
    """ Calculate angle from runway canter to the plane
        at the start point """
    (_x0, _y0) = get_xy_from_lat_lon(_lat0, _lon0)
    (_x, _y) = get_xy_from_lat_lon(_lat, _lon)

    delta_x = _x - _x0
    delta_y = _y - _y0

    if delta_x == 0:
        betta = 0
        gamma = 0
    else:
        angle_to_plane = math.degrees(math.atan(delta_y/delta_x))
        if delta_x < 0:
            # going to west
            angle_to_plane += 180
        # convert to azimuth
        gamma = (90 - angle_to_plane + 360) % 360
        # angle from runway center to direction to plane
        betta = alpha - gamma
        #print("angle_to_plane: {}, gamma: {}, betta: {}".format(angle_to_plane, gamma, betta))

    distance_from_start = math.sqrt(delta_x*delta_x + delta_y*delta_y)
    center_dev = distance_from_start * math.tan(math.radians(betta))
    return center_dev

def get_runway_center_correction(speed, center_dist):
    """ Get correction angle on runway """
    # Knots to m/s
    speed_meter_ps = speed * 0.514444
    # get the angle to center on railway in 100 sec
    center_correction = math.degrees(math.atan(center_dist/(speed_meter_ps*100 + 1)))
    #print("center_correction: {}".format(center_correction))
    return center_correction

def process_data(inputs):
    # pylint: disable=too-many-statements
    # pylint: disable=too-many-locals
    """ Main processing function """
    out = {}

    heading = float(inputs['Heading'])
    speed = float(inputs['Speed'])
    altitude = float(inputs['Altitude'])
    bank = float(inputs['Bank'])
    rpm = int(inputs['RPM'])
    latitude = float(inputs['Latitude'])
    longitude = float(inputs['Longitude'])

    print("State: {}".format(get_cur_state()))

    if get_cur_state()  == 'initial':
        InitialData['heading'] = heading
        InitialData['altitude'] = altitude
        # y, east
        InitialData['latitude'] = latitude
        # x, north
        InitialData['longitude'] = longitude

        SetPoints['altitude'] = InitialData['altitude']
        SetPoints['speed'] = Data['targetspeed']
        SetPoints['bank'] = 0.0
        SetPoints['heading'] = InitialData['heading']

        if rpm > Data['engine_on_rpm']:
            next_state()

    if get_cur_state() == 'takeoff':
        SetPoints['speed'] = Data['targetspeed']
        center_dist = get_runway_center_dist(InitialData['latitude'],
                          InitialData['longitude'],
                          latitude, longitude, InitialData['heading'])
        center_correction = get_runway_center_correction(speed, center_dist)
        SetPoints['heading'] = InitialData['heading'] + center_correction

        if speed > Data['takeoffspeed']:
            SetPoints['altitude'] = Data['targetalt']
            next_state()

    if get_cur_state() == 'climbing':
        if altitude > SetPoints['altitude']:
            next_state()

    if get_cur_state() == 'turn':
        if not get_cur_flag():
            SetPoints['heading'] = (SetPoints['heading'] + get_cur_arg()) % 360
            SetPoints['bank'] = math.copysign(Data['turnbank'], - get_cur_arg())
            set_cur_flag(True)
        if heading < SetPoints['heading'] + Data['turn_headingdelta'] \
           and SetPoints['heading'] < heading + Data['turn_headingdelta']:
            SetPoints['bank'] = 0
            next_state()

    if get_cur_state() == 'level':
        (_x0, _y0) = get_xy_from_lat_lon(latitude, longitude)
        (_xa, _ya) = get_cur_arg()
        (_x1, _y1) = get_x_y_from_xa_ya(_xa, _ya)
        SetPoints['heading'] = get_heading(_x0, _y0, _x1, _y1)
        distance = get_distance(_x0, _y0, _x1, _y1)
        print("Distance: {}, angle: {}".format(distance, SetPoints['heading']))
        if distance < Data['level_distancedelta']:
            next_state()
        # FIXME
        if SetPoints['heading'] > (heading + Data['level_maxangle']) % 360 or \
           (SetPoints['heading'] + Data['level_maxangle']) % 360 < heading:
            print("ERROR: can't get to the point.")
            time.sleep(100)

    heading_dev = heading - SetPoints['heading']
    out['rudder'] = process_heading(heading_dev)

    # Climb to preset altitude
    altitude_dev = SetPoints['altitude'] - altitude
    out['elevator'] = process_altitude(altitude_dev)

    bank_dev = bank - SetPoints['bank']
    out['aileron'] = process_bank(bank_dev)

    speed_dev = speed - SetPoints['speed']
    out['throttle'] = process_speed(speed_dev)

    print("Deviations: Heading {:+06.2f}, altitude {:+06.2f}, bank {:+06.2f}, speed {:+06.2f}"
          .format(heading_dev, altitude_dev, bank_dev, speed_dev))

    return out
