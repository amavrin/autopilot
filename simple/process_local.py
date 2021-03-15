""" Process data locally """

import time
import math
from simple_pid import PID

Data = {}
SetPoints = {}
InitialData = {}
PIDS = {}
# initial, takeoff, climbing, turn,
# setspeed
# level, descending, landing, stop
States = {}

def error_pause(_s,_t):
    """ Print message and sleep """
    print("ERROR: {}".format(_s))
    time.sleep(_t)

def init():
    # pylint: disable=too-many-statements
    """ Init data for local processing """
    Data['takeoffspeed'] = 60.0
    Data['prelanding_speed'] = 75.0
    Data['landing_speed'] = 30.0
    Data['targetspeed'] = 150.0
    Data['targetalt'] = 700.0
    Data['landingalt'] = 30.0
    Data['dropspeed_ground_alt'] = 3.0
    Data['glissadealt'] = 500.0
    Data['glissadespeed'] = 40.0
    Data['engine_on_rpm'] = 100
    Data['turnbank'] = 40
    Data['turn_headingdelta'] = 0.5
    Data['level_distancedelta'] = 40
    Data['level_maxangle'] = 45
    PIDS['rudder_runway'] = PID(0.005, 0.005, 0.001, setpoint=0)
    PIDS['rudder_runway'].output_limits = (-1.0, 1.0)
    PIDS['rudder_landing'] = PID(0.02, 0.01, 0.02, setpoint=0)
    PIDS['rudder_landing'].output_limits = (-0.5, 0.5)
    PIDS['rudder_flight'] = PID(0.01, 0.004, 0.001, setpoint=0)
    PIDS['rudder_flight'].output_limits = (-0.5, 0.5)
    PIDS['aileron_level'] = PID(0.04, 0.01, 0.01, setpoint=0)
    PIDS['aileron_level'].output_limits = (-0.4, 0.4)
    PIDS['aileron_turn'] = PID(0.01, 0.0, 0.0, setpoint=0)
    PIDS['aileron_turn'].output_limits = (-0.4, 0.4)
    PIDS['throttle'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['throttle'].output_limits = (0.005, 1.0)
    PIDS['elevator_alt'] = PID(0.002, 0.01, 0.01, setpoint=0)
    PIDS['elevator_alt'].output_limits = (-0.1, 0.01)
    PIDS['elevator_climb'] = PID(0.02, 0.0, 0.0, setpoint=0)
    PIDS['elevator_climb'].output_limits = (-0.1, 0.01)
    PIDS['elevator_pitch'] = PID(0.04, 0.02, 0.1, setpoint=0)
    PIDS['elevator_pitch'].output_limits = (-0.3, 0.1)
    States['current'] = 0
    States['program'] = []
    States['program'].append({ 'name': 'initial' })

    #States['program'].append({ 'name': 'setspeed', 'arg': 75 })
    #States['program'].append({ 'name': 'setalt', 'arg': 100 })
    #States['program'].append({ 'name': 'takeoff' })
    #States['program'].append({ 'name': 'climbing' })
    #States['program'].append({ 'name': 'setspeed', 'arg': Data['glissadespeed'] })
    #States['program'].append({ 'name': 'level', 'arg': (0, 2000) })
    #States['program'].append({ 'name': 'descending', 'arg': (0,2500) })
    #States['program'].append({ 'name': 'landing' })
    #States['program'].append({ 'name': 'stop' })

    States['program'].append({ 'name': 'setspeed', 'arg': Data['targetspeed'] })
    States['program'].append({ 'name': 'setalt', 'arg': Data['targetalt'] })
    States['program'].append({ 'name': 'takeoff' })
    States['program'].append({ 'name': 'climbing' })
    States['program'].append({ 'name': 'turn', 'arg': -90 })
    # 2000m right and 4000m back to start
    States['program'].append({ 'name': 'level', 'arg': (-700, -3000) })
    States['program'].append({ 'name': 'turn', 'arg': 90 })
    States['program'].append({ 'name': 'level', 'arg': (-700, -6000) })
    # Turn to the glissade, take off speed
    States['program'].append({ 'name': 'setspeed', 'arg': Data['prelanding_speed'] })
    States['program'].append({ 'name': 'turn', 'arg': -270 })
    # Lower to glissade start
    States['program'].append({ 'name': 'setalt', 'arg': Data['glissadealt'] })
    States['program'].append({ 'name': 'level', 'arg': (0, -3000) })
    # Make flight level and take off speed to glissage's one
    States['program'].append({ 'name': 'turn', 'arg': -270 })
    States['program'].append({ 'name': 'setspeed', 'arg': Data['glissadespeed'] })
    States['program'].append({ 'name': 'level', 'arg': (0, -2000) })
    # Adjust heading
    States['program'].append({ 'name': 'turn', 'arg': -270 })
    States['program'].append({ 'name': 'descending', 'arg': (0,0) })
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
    _k = 0.0
    if get_cur_state() in ('takeoff', 'stop'):
        rudder = PIDS['rudder_runway'](heading_dev)

    if get_cur_state() in ('descending', 'landing'):
        rudder = PIDS['rudder_landing'](heading_dev)

    if get_cur_state() in ('level', 'climbing', 'turn'):
        if abs(heading_dev) < 60:
            rudder = PIDS['rudder_flight'](heading_dev)
            if abs(heading_dev) > Data['turn_headingdelta']:
                # Heading error is average, gradually set bank
                #PIDS['rudder_flight'].auto_mode = False
                _k = 1.25 * math.atan(math.radians(- heading_dev))
        elif get_cur_state() == 'turn':
            # Set bank to turnbank with the respect to required direction
            _k = math.copysign(1, get_cur_arg())
        else:
            error_pause("Heading error is too large: {} and not turning"
                        .format(heading_dev), 100)

    SetPoints['bank'] = _k*Data['turnbank']
    return rudder

def process_bank(bank_dev):
    """ Sample bank processing """
    aileron = 0.0
    if get_cur_state() == 'turn':
        aileron = PIDS['aileron_turn'](bank_dev)
    elif get_cur_state() in ('level', 'climbing', 'descending', 'landing'):
        aileron = PIDS['aileron_level'](bank_dev)

    return aileron

def process_altitude(altitude_dev):
    """ Altitude processing during flight """
    elevator = 0.0
    if get_cur_state() not in ('takeoff', 'landing'):
        elevator = PIDS['elevator_alt'](- altitude_dev)
    return elevator

def process_climb(climb_dev):
    """ Altitude processing on glissade """
    elevator = PIDS['elevator_climb'](- climb_dev)
    return elevator

def process_pitch(pitch_dev):
    """ Pitch processing on landing """
    elevator = PIDS['elevator_pitch'](- pitch_dev)
    print("pitch_dev: {:+06.2f}, elevator: {:+06.2f}".format(pitch_dev, elevator))
    return elevator

def process_speed(speed_dev):
    """ Flight speed processing """
    return PIDS['throttle'](speed_dev)

def process_speed_landing(climb_dev):
    """ Landing speed processing """
    throttle = PIDS['throttle'](climb_dev)
    print("climb_dev: {}, throttle: {}".format(climb_dev, throttle))
    return throttle

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

def get_runway_center_dist(_lat, _lon, _a0):
    """ Calculate distance from runway center to the plane """
    # Starting point
    (_x0, _y0) = get_xy_from_xa_ya(0, 0)
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
    # get the angle to center on railway in 30 sec
    center_correction = math.degrees(math.atan(center_dist/(speed_meter_ps*30 + 1)))
    return center_correction

def get_runway_center_heading(_lat, _lon, _speed):
    """ Get heading to runway center """
    center_dist = get_runway_center_dist(_lat, _lon, InitialData['heading'])
    center_correction = get_runway_center_correction(_speed, center_dist)
    center_heading = InitialData['heading'] + center_correction
    return center_heading

def process_data(inputs):
    # pylint: disable=too-many-statements
    # pylint: disable=too-many-locals
    # pylint: disable=too-many-branches
    """ Main processing function """
    out = {}

    heading = float(inputs['Heading'])
    speed = float(inputs['Speed'])
    altitude = float(inputs['Altitude'])
    bank = float(inputs['Bank'])
    rpm = int(inputs['RPM'])
    latitude = float(inputs['Latitude'])
    longitude = float(inputs['Longitude'])
    climb = float(inputs['Climb'])
    pitch = float(inputs['Pitch'])
    elevation = float(inputs['Elevation'])

    ground_alt = altitude - elevation

    if not get_cur_flag():
        for pid in PIDS:
            PIDS[pid].reset()
        set_cur_flag(True)

    print("State: {}".format(get_cur_state()))

    if get_cur_state()  == 'initial':
        InitialData['heading'] = heading
        InitialData['altitude'] = altitude
        InitialData['elevation'] = elevation
        InitialData['ground_alt'] = ground_alt
        InitialData['latitude'] = latitude
        InitialData['longitude'] = longitude

        SetPoints['altitude'] = InitialData['altitude']
        SetPoints['bank'] = 0.0
        SetPoints['heading'] = InitialData['heading']
        SetPoints['speed'] = 0.0
        SetPoints['climb'] = 0.0
        SetPoints['flaps'] = 0.0
        SetPoints['pitch'] = 0.0

        if rpm > Data['engine_on_rpm']:
            next_state()

    if get_cur_state() == 'setalt':
        SetPoints['altitude'] = get_cur_arg()
        next_state()

    if get_cur_state() == 'setspeed':
        SetPoints['speed'] = get_cur_arg()
        next_state()

    if get_cur_state() == 'takeoff':
        SetPoints['heading'] = get_runway_center_heading(latitude, longitude, speed)

        if speed > Data['takeoffspeed']:
            next_state()

    if get_cur_state() == 'climbing':
        if altitude > SetPoints['altitude']:
            next_state()

    if get_cur_state() == 'turn':
        SetPoints['heading'] = abs(get_cur_arg())
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
            error_pause("can't get to the point", 100)

    if get_cur_state() == 'descending':
        # Calculate required vertical speed (feet per sec)

        # 1 knot is 1,68781 feet per sec
        speed_fps = speed * 1.68781
        # get distance-to-the-start-point/alt ratio. 1 meter is 3,28084 feets
        (_x, _y) = get_xy_from_lat_lon(latitude, longitude)
        (land_xa, land_ya) = get_cur_arg()
        (land_x, land_y) = get_xy_from_xa_ya(land_xa, land_ya)
        distance_to_landpoint = get_distance(_x, _y, land_x, land_y)
        dist_alt_ratio = distance_to_landpoint * 3.28084 / altitude
        climb_required = (-1) * speed_fps / dist_alt_ratio
        SetPoints['climb'] = climb_required

        SetPoints['heading'] = get_runway_center_heading(latitude, longitude, speed)

        print("to land: {:+08.2f}, heading: {:+05.2f}"
              .format(distance_to_landpoint, SetPoints['heading']))


        if ground_alt < 10 * Data['landingalt']:
            SetPoints['flaps'] = 0.75
            SetPoints['speed'] = Data['landing_speed']
            SetPoints['climb'] = climb_required / 3
        if ground_alt < Data['landingalt']:
            next_state()

    if get_cur_state() == 'landing':
        SetPoints['flaps'] = 1.0
        SetPoints['pitch'] = 7.0
        SetPoints['heading'] = get_runway_center_heading(latitude, longitude, speed)
        if ground_alt < InitialData['ground_alt'] + Data['dropspeed_ground_alt']:
            SetPoints['speed'] = 0
        if speed < 30:
            next_state()

    if get_cur_state() == 'stop':
        print("Brakes!")

    heading_dev = get_heading_diff(SetPoints['heading'], heading)
    out['rudder'] = process_heading(heading_dev)

    altitude_dev = altitude - SetPoints['altitude']
    climb_dev = climb - SetPoints['climb']
    pitch_dev = pitch - SetPoints['pitch']
    if get_cur_state() not in ('descending', 'landing'):
        out['elevator'] = process_altitude(altitude_dev)
    if get_cur_state() == 'descending':
        out['elevator'] = process_climb(climb_dev)
    if get_cur_state() == 'landing':
        out['elevator'] = process_pitch(pitch_dev)

    bank_dev = bank - SetPoints['bank']
    out['aileron'] = process_bank(bank_dev)

    speed_dev = speed - SetPoints['speed']
    out['throttle'] = process_speed(speed_dev)

    out['flaps'] = SetPoints['flaps']

    print("Deviations: Heading {:+06.2f}, altitude {:+06.2f},"
          "bank {:+06.2f}, speed {:+06.2f}, climb {:+06.2f}, pitch: {:+06.2f}"
          .format(heading_dev, altitude_dev, bank_dev, speed_dev, climb_dev, pitch_dev))

    return out

if __name__ == "__main__":
    _d = get_distance(-3976.07,+3.18,-4000.00,+0.00)
    print(_d)
