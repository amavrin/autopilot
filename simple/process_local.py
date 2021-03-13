""" Process data locally """

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
    Data['targetalt'] = 700.0
    Data['targetspeed'] = 100.0
    Data['engine_on_rpm'] = 100
    Data['turnbank'] = 15
    Data['turn_headingdelta'] = 3
    Data['level_distancedelta'] = 30
    Data['level_maxangle'] = 45
    PIDS['rudder'] = PID(0.01, 0.005, 0.001, setpoint=0)
    PIDS['rudder'].output_limits = (-1.0, 1.0)
    PIDS['aileron_level'] = PID(0.01, 0.03, 0.01, setpoint=0)
    PIDS['aileron_level'].output_limits = (-0.4, 0.4)
    PIDS['throttle'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['throttle'].output_limits = (0.3, 1.0)
    PIDS['elevator'] = PID(0.001, 0.01, 0.01, setpoint=0)
    PIDS['elevator'].output_limits = (-0.1, 0.01)
    States['current'] = 0
    States['program'] = []
    States['program'].append({ 'name': 'initial' })
    States['program'].append({ 'name': 'takeoff' })
    States['program'].append({ 'name': 'climbing' })
    States['program'].append({ 'name': 'turn', 'arg': -90 })
    # 2000m right to start
    States['program'].append({ 'name': 'level', 'arg': (-2000, 0) })
    # 2000m right and 4000m back to start
    States['program'].append({ 'name': 'level', 'arg': (-2000, -3000) })
    States['program'].append({ 'name': 'turn', 'arg': -270 })
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
    """ Calculate relative X and Y based on latitude and longitude """
    r_lat = math.radians(_lat - InitialData['latitude'])
    r_lon = math.radians(_lon - InitialData['longitude'])
    _x = 6371000 * r_lon * math.cos(r_lat)
    _y = 6371000 * r_lat
    return (_x, _y)

def get_lat_lon_from_xy(_x, _y):
    """ Calculate lat and lon from X and Y """
    r_lat = _y / 6371000
    r_lon = _x / 6371000 / math.cos(r_lat)
    lat = math.degrees(r_lat) + InitialData['latitude']
    lon = math.degrees(r_lon) + InitialData['longitude']
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
    """ Calculate heading difference """
    _h1 = _h1 % 360
    _h2 = _h2 % 360
    hdiff = _h2 - _h1 + (_h2 < _h1) * 360
    if hdiff > 180:
        hdiff -= 360
    return hdiff

def get_runway_center_dist(_lat0, _lon0, _lat, _lon, _a0):
    """ Calculate distance from runway center to the plane """
    # Starting point
    (_x0, _y0) = get_xy_from_lat_lon(_lat0, _lon0)
    # Current plane point
    (_x, _y) = get_xy_from_lat_lon(_lat, _lon)

    delta_x = _x - _x0
    delta_y = _y - _y0

    r_angle_to_plane = math.atan2(delta_y, delta_x)
    angle_to_plane = math.degrees(r_angle_to_plane)
    _a1 = angle_to_heading(angle_to_plane)
    # angle from runway center to direction to plane
    _a_delta = get_heading_diff(_a1, _a0)

    distance_from_start = get_distance(_x0, _y0, _x, _y)
    center_dev = distance_from_start * math.tan(math.radians(_a_delta))
    return center_dev

def get_runway_center_correction(speed, center_dist):
    """ Get correction angle on runway """
    # Knots to m/s
    speed_meter_ps = speed * 0.514444
    # get the angle to center on railway in 100 sec
    center_correction = math.degrees(math.atan(center_dist/(speed_meter_ps*100 + 1)))
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
        InitialData['latitude'] = latitude
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
        SetPoints['heading'] = abs(get_cur_arg())
        SetPoints['bank'] = math.copysign(Data['turnbank'], get_cur_arg())
        heading_diff = get_heading_diff(heading, SetPoints['heading'])
        if abs(heading_diff) < Data['turn_headingdelta']:
            SetPoints['bank'] = 0
            next_state()

    if get_cur_state() == 'level':
        (_x, _y) = get_xy_from_lat_lon(latitude, longitude)
        (_xa, _ya) = get_cur_arg()
        (_x1, _y1) = get_xy_from_xa_ya(_xa, _ya)
        SetPoints['heading'] = get_heading(_x, _y, _x1, _y1)

        distance = get_distance(_x, _y, _x1, _y1)
        heading_error = get_heading_diff(heading, SetPoints['heading'])
        print("Distance: {}, heading_error: {}".format(distance, heading_error))
        if distance < Data['level_distancedelta']:
            next_state()
        if abs(heading_error) > Data['level_maxangle']:
            print("ERROR: can't get to the point.")

    heading_dev = get_heading_diff(SetPoints['heading'], heading)
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

if __name__ == "__main__":
    InitialData['heading'] = 270
    InitialData['altitude'] = 0.0
    # Palaca sqare 59.939018, 30.316177
    InitialData['latitude'] = 59.939018
    InitialData['longitude'] = 30.316177
    # Admitalty 59.939055, 30.306933
    ADM_LAT = 59.939055
    ADM_LON = 30.306933
    (_x0, _y0) = get_xy_from_lat_lon(InitialData['latitude'],
                                     InitialData['longitude'])
    (_x, _y) = get_xy_from_xa_ya(-1000, 0)
    _h = get_heading(_x0, _y0, _x, _y)
    print(_h)
    (_x1, _y1) = get_xy_from_lat_lon(ADM_LAT, ADM_LON)
    _h = get_heading(_x1, _y1, _x, _y)
    print(_h)
