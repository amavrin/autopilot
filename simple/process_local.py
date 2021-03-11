""" Process data locally """

import math
import time
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
    Data['targetalt'] = 300.0
    Data['targetspeed'] = 100.0
    Data['engine_on_rpm'] = 100
    Data['turnbank'] = 15
    Data['turn_headingdelta'] = 2
    PIDS['rudder_runway'] = PID(0.01, 0.005, 0.001, setpoint=0)
    PIDS['aileron_level'] = PID(0.01, 0.03, 0.01, setpoint=0)
    PIDS['aileron_level'].output_limits = (-0.4, 0.4)
    PIDS['throttle'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['throttle'].output_limits = (0.3, 1.0)
    PIDS['elevator_level'] = PID(0.001, 0.0, 0.01, setpoint=0)
    PIDS['elevator_level'].output_limits = (-0.05, 0.01)
    PIDS['elevator_turn'] = PID(0.001, 0.0, 0.01, setpoint=0)
    PIDS['elevator_turn'].output_limits = (-0.2, 0.05)
    States['current'] = 0
    States['program'] = []
    States['program'].append({ 'name': 'initial' })
    States['program'].append({ 'name': 'takeoff' })
    States['program'].append({ 'name': 'climbing' })
    States['program'].append({ 'name': 'turn', 'arg': 180 })
    States['program'].append({ 'name': 'level' })
    States['program'].append({ 'name': 'turn', 'arg': 180 })
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

def process_heading(heading_dev, _state):
    """ Sample heading processing """
    rudder = 0.0
    if _state == 'takeoff':
        # During takeoff, rule a rudder
        rudder = PIDS['rudder_runway'](heading_dev)
    return rudder

def process_bank(bank_dev, _state):
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

def process_altitude(altitude_dev, _state):
    """ Sample altitude processing """
    elevator = 0.0
    if get_cur_state() == 'turn':
        elevator = PIDS['elevator_turn'](altitude_dev)
    else:
        elevator = PIDS['elevator_level'](altitude_dev)
    return elevator

def process_speed(speed_dev, _state):
    """ Sample speed processing """
    return PIDS['throttle'](speed_dev)

def get_xy_from_lat_lon(_lat, _lon):
    """ Calculate X and Y based on latitude and longitude """
    r_lat = math.radians(_lat)
    r_lon = math.radians(_lon)
    _x = 6371000 * r_lon * math.cos(r_lat)
    _y = 6371000 * r_lat
    return (_x, _y)

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
            SetPoints['bank'] = math.copysign(Data['turnbank'], get_cur_arg())
            set_cur_flag(True)
        if heading < SetPoints['heading'] + Data['turn_headingdelta'] \
           and SetPoints['heading'] < heading + Data['turn_headingdelta']:
            SetPoints['bank'] = 0
            next_state()

    heading_dev = heading - SetPoints['heading']
    out['rudder'] = process_heading(heading_dev, get_cur_state())

    # Climb to preset altitude
    altitude_dev = SetPoints['altitude'] - altitude
    out['elevator'] = process_altitude(altitude_dev, get_cur_state())

    bank_dev = bank - SetPoints['bank']
    out['aileron'] = process_bank(bank_dev, get_cur_state())

    speed_dev = speed - SetPoints['speed']
    out['throttle'] = process_speed(speed_dev, get_cur_state())

    print("Deviations: Heading {:+06.2f}, altitude {:+06.2f}, bank {:+06.2f}, speed {:+06.2f}"
          .format(heading_dev, altitude_dev, bank_dev, speed_dev))

    return out
