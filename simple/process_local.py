""" Process data locally """

from simple_pid import PID

SetPoints = {}
PIDS = {}

def init():
    """ Init data for local processing """
    SetPoints['takeoffspeed'] = 40.0
    SetPoints['targetalt'] = 300.0
    SetPoints['targetspeed'] = 100.0
    PIDS['heading'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['bank'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['speed'] = PID(0.01, 0.003, 0.01, setpoint=0)
    PIDS['speed'].output_limits = (0.0, 1.0)
    PIDS['altitude'] = PID(0.002, 0.0001, 0.04, setpoint=0)
    PIDS['altitude'].output_limits = (-0.05, 0.01)

def process_heading(heading_dev):
    """ Sample heading processing """
    return PIDS['heading'](heading_dev)

def process_bank(bank):
    """ Sample bank processing """
    return PIDS['bank'](bank)

def process_altitude(altitude_dev):
    """ Sample altitude processing """
    return PIDS['altitude'](altitude_dev)

def process_speed(speed_dev):
    """ Sample speed processing """
    return PIDS['speed'](speed_dev)

def process_data(inputs):
    """ Main processing function """
    out = {}

    heading = float(inputs['Heading'])
    speed = float(inputs['Speed'])
    altitude = float(inputs['Altitude'])
    bank = float(inputs['Bank'])

    if 'heading' not in SetPoints:
        SetPoints['heading'] = heading
        print("heading: {}".format(SetPoints['heading']))

    heading_dev = heading - SetPoints['heading']
    out['rudder'] = process_heading(heading_dev)

    # Climb to preset altitude
    altitude_dev = SetPoints['targetalt'] - altitude
    if speed <= SetPoints['takeoffspeed']:
        out['elevator'] = 0.0
    else:
        out['elevator'] = process_altitude(altitude_dev)

    out['aileron'] = process_bank(bank)

    speed_dev = speed - SetPoints['targetspeed']
    out['throttle'] = process_speed(speed_dev)

    print("Deviations: Heading {:+06.2f}, altitude {:+06.2f}, bank {:+06.2f}, speed {:+06.2f}"
          .format(heading_dev, altitude_dev, bank, speed_dev))

    return out
